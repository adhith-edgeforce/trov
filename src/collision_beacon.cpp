/**
 * collision_beacon.cpp
 *
 * Reads /points (PointCloud2) and checks if any point falls inside a
 * CYLINDER ROI centred on the lidar origin.
 *
 *   Cylinder check:
 *     sqrt(x² + y²) <= CYLINDER_RADIUS   (horizontal distance from lidar)
 *     z >= Z_MIN && z <= Z_MAX           (height band — rejects ground & sky)
 *
 * If MIN_POINTS or more points are inside → GPIO HIGH (beacon ON)
 * Otherwise                               → GPIO LOW  (beacon OFF)
 *
 * ROI dimensions (tune these at the top):
 *   CYLINDER_RADIUS : 0.50 m  (horizontal radius from lidar centre)
 *   Z_MIN           : 0.10 m  (floor — rejects ground returns)
 *   Z_MAX           : 2.00 m  (ceiling — ankle to head)
 *
 * RViz marker published on /collision_roi_marker
 *   GREEN (semi-transparent) = clear
 *   RED   (semi-transparent) = obstacle detected
 *   Add in RViz: Add → By topic → /collision_roi_marker (Marker)
 *   Fixed Frame: rslidar  (frame_id from the lidar driver)
 *
 * GPIO mapping (Jetson AGX Orin):
 *   gpiochip1, line 9 = PBB.01 = physical pin 16
 *
 * Build: colcon build --packages-select trov
 * Run:   sudo ros2 run trov collision_beacon
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <gpiod.h>
#include <cmath>

// ── GPIO config ──────────────────────────────────────────────────────────────
#define GPIO_CHIP    "gpiochip1"
#define GPIO_LINE    9          // PBB.01 = physical pin 16 on Jetson AGX Orin

// ── Cylinder ROI (lidar_link / rslidar frame) ─────────────────────────────
//    All values in metres.  Change these to retune without touching any logic.
#define CYLINDER_RADIUS   0.30f   // horizontal radius from lidar origin
#define Z_MIN             0.10f   // ignore returns below this (ground rejection)
#define Z_MAX             2.50f   // ignore returns above this

#define MIN_POINTS        7       // detections needed to trigger alarm

// ── Debug ─────────────────────────────────────────────────────────────────
#define DEBUG_EVERY_N     30      // print stats every N callbacks


class CollisionBeaconNode : public rclcpp::Node
{
public:
  CollisionBeaconNode()
  : Node("collision_beacon_node"),
    alarm_active_(false),
    chip_(nullptr),
    line_(nullptr),
    cb_count_(0)
  {
    // ── GPIO setup ────────────────────────────────────────────────────────
    chip_ = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip_) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", GPIO_CHIP);
      return;
    }

    line_ = gpiod_chip_get_line(chip_, GPIO_LINE);
    if (!line_) {
      RCLCPP_ERROR(get_logger(), "Failed to get line %d", GPIO_LINE);
      gpiod_chip_close(chip_);
      chip_ = nullptr;
      return;
    }

    struct gpiod_line_request_config cfg;
    cfg.consumer     = "collision_beacon";
    cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
    cfg.flags        = 0;

    if (gpiod_line_request(line_, &cfg, 0) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to request GPIO line as output");
      line_ = nullptr;
      gpiod_chip_close(chip_);
      chip_ = nullptr;
      return;
    }

    RCLCPP_INFO(get_logger(),
      "GPIO ready — chip=%s line=%d (PBB.01 / physical pin 16)",
      GPIO_CHIP, GPIO_LINE);

    // ── Subscriber ────────────────────────────────────────────────────────
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points",
      rclcpp::SensorDataQoS(),
      std::bind(&CollisionBeaconNode::cloud_callback, this, std::placeholders::_1)
    );

    // ── Marker publisher ──────────────────────────────────────────────────
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "collision_roi_marker", 10
    );

    // Publish marker at 2 Hz so RViz always has it
    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CollisionBeaconNode::publish_roi_marker, this)
    );

    RCLCPP_INFO(get_logger(), "CollisionBeaconNode ready — watching /points");
    RCLCPP_INFO(get_logger(),
      "Cylinder ROI: radius=%.2fm  z[%.2f → %.2fm]  min_points=%d",
      CYLINDER_RADIUS, Z_MIN, Z_MAX, MIN_POINTS);
    RCLCPP_INFO(get_logger(), "ROI marker  → /collision_roi_marker");
    RCLCPP_INFO(get_logger(), "──────────────────────────────────────────");
    RCLCPP_INFO(get_logger(), "DEBUG MODE ON — printing every %d callbacks", DEBUG_EVERY_N);
    RCLCPP_INFO(get_logger(), "──────────────────────────────────────────");
  }

  ~CollisionBeaconNode()
  {
    RCLCPP_INFO(get_logger(), "Shutting down — GPIO LOW");
    gpio_set(0);
    if (line_) gpiod_line_release(line_);
    if (chip_) gpiod_chip_close(chip_);
  }

private:

  // ── Point cloud callback ─────────────────────────────────────────────────
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    cb_count_++;

    // Grab frame_id once so the marker stays in the right frame
    if (current_frame_id_ != msg->header.frame_id) {
      current_frame_id_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(),
        "Marker frame_id set to '%s'", current_frame_id_.c_str());
    }

    // First cloud diagnostics
    if (cb_count_ == 1) {
      RCLCPP_INFO(get_logger(),
        "[DEBUG] ✔ First cloud — frame='%s'  total_points=%u  point_step=%u",
        msg->header.frame_id.c_str(),
        msg->width * msg->height,
        msg->point_step);
      RCLCPP_INFO(get_logger(), "[DEBUG] Fields:");
      for (const auto & f : msg->fields) {
        RCLCPP_INFO(get_logger(), "[DEBUG]   '%s' offset=%u datatype=%u",
          f.name.c_str(), f.offset, f.datatype);
      }
    }

    int total_pts  = 0;
    int nan_pts    = 0;
    int in_roi     = 0;

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz)
    {
      const float x = *ix;
      const float y = *iy;
      const float z = *iz;

      total_pts++;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        nan_pts++;
        continue;
      }

      // ── Cylinder test ──────────────────────────────────────────────────
      // Check z band first (cheap), then horizontal radius (needs multiply)
      // Avoids sqrt by comparing squared distances
      if (z >= Z_MIN && z <= Z_MAX &&
          (x * x + y * y) <= (CYLINDER_RADIUS * CYLINDER_RADIUS))
      {
        in_roi++;
      }
    }

    bool obstacle = (in_roi >= MIN_POINTS);

    // Periodic debug dump
    if (cb_count_ % DEBUG_EVERY_N == 0) {
      RCLCPP_INFO(get_logger(),
        "[DEBUG #%04d] total=%d  nan=%d  in_cylinder=%d  alarm=%s",
        cb_count_, total_pts, nan_pts, in_roi,
        alarm_active_ ? "ON" : "OFF");
    }

    // ── State machine ─────────────────────────────────────────────────────
    if (obstacle && !alarm_active_) {
      RCLCPP_WARN(get_logger(),
        "[BEACON ON]  %d points in cylinder ROI  (cb #%d)", in_roi, cb_count_);
      alarm_active_ = true;
      gpio_set(1);

    } else if (!obstacle && alarm_active_) {
      RCLCPP_INFO(get_logger(),
        "[BEACON OFF] cylinder clear  (cb #%d)", cb_count_);
      alarm_active_ = false;
      gpio_set(0);
    }
  }

  // ── RViz marker ──────────────────────────────────────────────────────────
  //
  // RViz's CYLINDER marker is aligned along its Z axis — stands upright.
  //
  // scale.x = scale.y = diameter  (2 × radius)
  // scale.z = height of the cylinder  (Z_MAX - Z_MIN)
  //
  // pose.position.z is shifted to the vertical centre of the height band
  // so the marker aligns exactly with the ROI.
  void publish_roi_marker()
  {
    visualization_msgs::msg::Marker m;

    m.header.stamp    = now();
    m.header.frame_id = current_frame_id_.empty() ? "rslidar" : current_frame_id_;

    m.ns     = "collision_beacon";
    m.id     = 0;
    m.type   = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    // Centre the cylinder vertically in the height band
    m.pose.position.x    = 0.0;
    m.pose.position.y    = 0.0;
    m.pose.position.z    = (Z_MIN + Z_MAX) / 2.0;
    m.pose.orientation.w = 1.0;

    // diameter = 2 × radius,  height = Z_MAX - Z_MIN
    m.scale.x = 2.0 * CYLINDER_RADIUS;
    m.scale.y = 2.0 * CYLINDER_RADIUS;
    m.scale.z = Z_MAX - Z_MIN;

    // GREEN = clear, RED = alarm — both semi-transparent
    if (alarm_active_) {
      m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.40f;
    } else {
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.20f;
    }

    m.lifetime = rclcpp::Duration(0, 0);  // never auto-delete

    marker_pub_->publish(m);
  }

  // ── GPIO helper ───────────────────────────────────────────────────────────
  void gpio_set(int value)
  {
    if (!line_) {
      RCLCPP_WARN_ONCE(get_logger(), "GPIO not initialised — skipping");
      return;
    }
    if (gpiod_line_set_value(line_, value) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to set GPIO to %d", value);
      return;
    }
    RCLCPP_INFO(get_logger(),
      "[GPIO] line %d → %s", GPIO_LINE, value ? "HIGH ▲" : "LOW  ▼");
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  bool        alarm_active_;
  gpiod_chip* chip_;
  gpiod_line* line_;
  int         cb_count_;
  std::string current_frame_id_;
};


// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionBeaconNode>());
  rclcpp::shutdown();
  return 0;
}
