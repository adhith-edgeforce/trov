// bumper_filter_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BumperFilterNode : public rclcpp::Node
{
public:
  BumperFilterNode() : Node("bumper_filter_node")
  {
    // Declare parameters — tune these at runtime
    this->declare_parameter("input_topic",  "/points");
    this->declare_parameter("output_topic", "/points_filtered");
    this->declare_parameter("target_frame", "base_link");

    // Mask box in base_link frame (metres)
    // x = forward, y = left, z = up
    // Covers the bumper region directly in front of the lidar
    this->declare_parameter("mask_x_min",  1.10);   // how close to body centre
    this->declare_parameter("mask_x_max",  1.65);   // past the bumper face
    this->declare_parameter("mask_y_min", -1.55);   // half-width of bumper
    this->declare_parameter("mask_y_max",  1.55);
    this->declare_parameter("mask_z_min",  1.00);   // ground clearance
    this->declare_parameter("mask_z_max",  1.30);   // bumper top edge

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto input  = this->get_parameter("input_topic").as_string();
    auto output = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&BumperFilterNode::cloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Bumper filter ready: %s → %s", input.c_str(), output.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto target_frame = this->get_parameter("target_frame").as_string();

    // Transform entire cloud to base_link so mask coords are intuitive
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    try {
      tf_buffer_->transform(*msg, cloud_transformed, target_frame,
                            tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF transform failed: %s", ex.what());
      pub_->publish(*msg);   // pass through unfiltered rather than drop
      return;
    }

    // Read mask params
    double xmin = this->get_parameter("mask_x_min").as_double();
    double xmax = this->get_parameter("mask_x_max").as_double();
    double ymin = this->get_parameter("mask_y_min").as_double();
    double ymax = this->get_parameter("mask_y_max").as_double();
    double zmin = this->get_parameter("mask_z_min").as_double();
    double zmax = this->get_parameter("mask_z_max").as_double();

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_transformed, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.header = pcl_cloud.header;

    for (const auto & pt : pcl_cloud.points) {
      // Skip NaN points
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      // If point is inside the bumper box → drop it
      bool in_mask = (pt.x >= xmin && pt.x <= xmax &&
                      pt.y >= ymin && pt.y <= ymax &&
                      pt.z >= zmin && pt.z <= zmax);
      if (!in_mask) {
        filtered.points.push_back(pt);
      }
    }

    filtered.width    = filtered.points.size();
    filtered.height   = 1;
    filtered.is_dense = false;

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(filtered, out);
    out.header = cloud_transformed.header;   // keep base_link frame
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BumperFilterNode>());
  rclcpp::shutdown();
  return 0;
}
