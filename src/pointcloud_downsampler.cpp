/**
 * pointcloud_downsampler.cpp
 * --------------------------
 * Part of the `trov` package.
 * Voxel-grid + ratio downsampling of PointCloud2 for Foxglove visualisation.
 *
 * Parameters
 * ----------
 *   input_topic      (string,  default: /points)       — source cloud
 *   output_topic     (string,  default: /points_down)  — decimated cloud
 *
 *   voxel_size       (double,  default: 0.10)   Metres per voxel cell.
 *                                               Larger  → fewer points (coarser).
 *                                               Smaller → more  points (finer).
 *                                               0.0 = voxel filter disabled.
 *
 *   downsample_ratio (double,  default: 1.0)   Fraction of points to keep
 *                                               AFTER the voxel filter.
 *                                               1.0 = keep all.
 *                                               0.3 = keep 30 % (random).
 *
 *   max_points       (int,     default: 15000)  Hard output cap. 0 = no cap.
 *   publish_rate     (double,  default: 0.0)    Hz throttle. 0 = every frame.
 *
 * Build
 * -----
 *   colcon build --packages-select trov
 *
 * Run examples
 * ------------
 *   ros2 run trov pointcloud_downsampler
 *
 *   ros2 run trov pointcloud_downsampler \
 *     --ros-args -p voxel_size:=0.15 -p downsample_ratio:=0.4 \
 *                -p max_points:=8000  -p publish_rate:=5.0
 *
 * Tune live (no restart needed)
 * ------------------------------
 *   ros2 param set /pointcloud_downsampler voxel_size 0.2
 *   ros2 param set /pointcloud_downsampler downsample_ratio 0.3
 */

#include <chrono>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using PC2    = sensor_msgs::msg::PointCloud2;

class PointCloudDownsampler : public rclcpp::Node
{
public:
  PointCloudDownsampler()
  : Node("pointcloud_downsampler"), rng_(std::random_device{}())
  {
    // ── parameters ────────────────────────────────────────────────────────
    declare_parameter<std::string>("input_topic",      "/points_raw");
    declare_parameter<std::string>("output_topic",     "/points_down");
    declare_parameter<double>     ("voxel_size",       0.10);
    declare_parameter<double>     ("downsample_ratio", 1.0);
    declare_parameter<int>        ("max_points",       15000);
    declare_parameter<double>     ("publish_rate",     0.0);

    input_topic_      = get_parameter("input_topic").as_string();
    output_topic_     = get_parameter("output_topic").as_string();
    voxel_size_       = get_parameter("voxel_size").as_double();
    downsample_ratio_ = clamp_ratio(get_parameter("downsample_ratio").as_double());
    max_points_       = get_parameter("max_points").as_int();
    publish_rate_     = get_parameter("publish_rate").as_double();

    // ── live parameter updates ─────────────────────────────────────────────
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        for (const auto & p : params) {
          if (p.get_name() == "voxel_size")        voxel_size_       = p.as_double();
          if (p.get_name() == "downsample_ratio")  downsample_ratio_ = clamp_ratio(p.as_double());
          if (p.get_name() == "max_points")        max_points_       = p.as_int();
          if (p.get_name() == "publish_rate")      publish_rate_     = p.as_double();
        }
        RCLCPP_INFO(get_logger(),
          "[trov] Downsampler updated → voxel=%.3f m  ratio=%.3f  max=%d  rate=%.1f Hz",
          voxel_size_, downsample_ratio_, max_points_, publish_rate_);
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        return r;
      });

    // ── QoS: BEST_EFFORT depth-1 matches most LiDAR drivers ──────────────
    auto qos = rclcpp::QoS(1).best_effort();

    sub_ = create_subscription<PC2>(
      input_topic_, qos,
      std::bind(&PointCloudDownsampler::callback, this, std::placeholders::_1));

    pub_ = create_publisher<PC2>(output_topic_, qos);

    RCLCPP_INFO(get_logger(),
      "\n[trov] PointCloud Downsampler ready"
      "\n  %s  →  %s"
      "\n  voxel_size      = %.3f m   (0.0 = disabled)"
      "\n  downsample_ratio= %.3f     (1.0 = keep all, 0.3 = keep 30%%)"
      "\n  max_points      = %d       (0 = no cap)"
      "\n  publish_rate    = %.1f Hz  (0.0 = every frame)"
      "\n  Tune live: ros2 param set /pointcloud_downsampler voxel_size 0.2",
      input_topic_.c_str(), output_topic_.c_str(),
      voxel_size_, downsample_ratio_, max_points_, publish_rate_);
  }

private:
  // ── helpers ───────────────────────────────────────────────────────────────
  static double clamp_ratio(double v) { return std::max(0.001, std::min(1.0, v)); }

  // ── main callback ─────────────────────────────────────────────────────────
  void callback(const PC2::SharedPtr msg)
  {
    // Rate throttle
    if (publish_rate_ > 0.0) {
      auto now     = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - last_pub_).count();
      if (elapsed < 1.0 / publish_rate_) return;
      last_pub_ = now;
    }

    auto t0 = std::chrono::steady_clock::now();

    // 1. ROS → PCL
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    const size_t n_in = cloud->size();

    // 2. Voxel grid filter
    if (voxel_size_ > 0.0) {
      CloudT::Ptr tmp(new CloudT);
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(
        static_cast<float>(voxel_size_),
        static_cast<float>(voxel_size_),
        static_cast<float>(voxel_size_));
      vg.filter(*tmp);
      cloud = tmp;
    }

    // 3. Ratio decimation (random, O(keep) Fisher-Yates)
    if (downsample_ratio_ < 1.0) {
      size_t keep = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(cloud->size() * downsample_ratio_)));

      if (keep < cloud->size()) {
        std::vector<size_t> idx(cloud->size());
        std::iota(idx.begin(), idx.end(), 0);
        for (size_t i = 0; i < keep; ++i) {
          std::uniform_int_distribution<size_t> dist(i, idx.size() - 1);
          std::swap(idx[i], idx[dist(rng_)]);
        }
        CloudT::Ptr tmp(new CloudT);
        tmp->reserve(keep);
        for (size_t i = 0; i < keep; ++i) tmp->push_back((*cloud)[idx[i]]);
        tmp->width    = static_cast<uint32_t>(keep);
        tmp->height   = 1;
        tmp->is_dense = cloud->is_dense;
        cloud = tmp;
      }
    }

    // 4. Hard point cap
    if (max_points_ > 0 && static_cast<int>(cloud->size()) > max_points_) {
      std::vector<size_t> idx(cloud->size());
      std::iota(idx.begin(), idx.end(), 0);
      std::shuffle(idx.begin(), idx.end(), rng_);
      idx.resize(max_points_);

      CloudT::Ptr tmp(new CloudT);
      tmp->reserve(max_points_);
      for (size_t i : idx) tmp->push_back((*cloud)[i]);
      tmp->width    = static_cast<uint32_t>(max_points_);
      tmp->height   = 1;
      tmp->is_dense = cloud->is_dense;
      cloud = tmp;
    }

    // 5. PCL → ROS and publish
    PC2 out;
    pcl::toROSMsg(*cloud, out);
    out.header = msg->header;
    pub_->publish(out);

    double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "[trov] pointcloud_downsampler: %zu → %zu pts  (%.2f ms)  "
      "[voxel=%.3f m  ratio=%.2f  max=%d]",
      n_in, cloud->size(), ms, voxel_size_, downsample_ratio_, max_points_);
  }

  // ── members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<PC2>::SharedPtr sub_;
  rclcpp::Publisher<PC2>::SharedPtr    pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::string input_topic_, output_topic_;
  double      voxel_size_, downsample_ratio_, publish_rate_;
  int         max_points_;

  std::mt19937                          rng_;
  std::chrono::steady_clock::time_point last_pub_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudDownsampler>());
  rclcpp::shutdown();
  return 0;
}
