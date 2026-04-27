/**
 * sensor_health_monitor.cpp
 *
 * ROS2 Humble — C++ (rclcpp)
 *
 * Monitors three sensor topics and publishes individual boolean health
 * status for each, plus one overall topic.
 *
 * Subscribed topics:
 *   /points                          [sensor_msgs/PointCloud2]
 *   /imu/data                        [sensor_msgs/Imu]
 *   /mavros/global_position/global   [sensor_msgs/NavSatFix]
 *
 * Published topics:
 *   /sensor_health/lidar             [std_msgs/Bool]  true = healthy
 *   /sensor_health/imu               [std_msgs/Bool]  true = healthy
 *   /sensor_health/gps               [std_msgs/Bool]  true = healthy
 *   /sensor_health/all               [std_msgs/Bool]  true = ALL healthy
 *
 * A sensor is healthy if a message arrived within its timeout window.
 * A sensor that has NEVER been seen publishes false.
 *
 * UI integration:
 *   Subscribe to any of the four topics above — each is a plain Bool.
 *   No parsing needed; just read .data.
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
// To promote these to ROS2 parameters later, use declare_parameter() in the
// constructor and get_parameter() to read them back.

static constexpr double TIMEOUT_LIDAR_SEC = 1.0;   // /points
static constexpr double TIMEOUT_IMU_SEC   = 0.5;   // /imu/data
static constexpr double TIMEOUT_GPS_SEC   = 1.0;   // /mavros/global_position/global
static constexpr double PUBLISH_RATE_HZ   = 2.0;   // how often to publish health topics
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// SensorState — tracks health for a single topic
// ---------------------------------------------------------------------------
struct SensorState
{
  std::string display_name;   // used in log messages
  std::string topic;          // used in log messages
  double      timeout_sec;

  bool          ever_received{false};
  rclcpp::Time  last_received;

  // Call this inside every subscriber callback
  void on_message(const rclcpp::Time & now)
  {
    last_received = now;
    ever_received = true;
  }

  // Returns true if the sensor is currently healthy
  bool is_healthy(const rclcpp::Time & now) const
  {
    if (!ever_received) {
      return false;
    }
    return (now - last_received).seconds() <= timeout_sec;
  }
};


// ---------------------------------------------------------------------------
// SensorHealthMonitor — the ROS2 node
// ---------------------------------------------------------------------------
class SensorHealthMonitor : public rclcpp::Node
{
public:
  SensorHealthMonitor()
  : Node("sensor_health_monitor")
  {
    // ------------------------------------------------------------------
    // Initialise sensor states
    // ------------------------------------------------------------------
    lidar_state_.display_name = "LiDAR";
    lidar_state_.topic        = "/points";
    lidar_state_.timeout_sec  = TIMEOUT_LIDAR_SEC;

    imu_state_.display_name   = "IMU";
    imu_state_.topic          = "/imu/data";
    imu_state_.timeout_sec    = TIMEOUT_IMU_SEC;

    gps_state_.display_name   = "GPS";
    gps_state_.topic          = "/mavros/global_position/global";
    gps_state_.timeout_sec    = TIMEOUT_GPS_SEC;

    // ------------------------------------------------------------------
    // Subscribers
    // ------------------------------------------------------------------
    // SensorDataQoS = best-effort, small history — standard for sensor streams.
    // GPS uses reliable QoS because MAVROS typically publishes that way.
    // Mismatched QoS causes silent "no data" — keep these as-is unless you
    // know your publisher uses a different policy.
    auto sensor_qos = rclcpp::SensorDataQoS();

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        lidar_state_.on_message(this->now());
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_state_.on_message(this->now());
      });

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::QoS(10),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gps_state_.on_message(this->now());
      });

    // ------------------------------------------------------------------
    // Publishers — one per sensor + one overall
    // ------------------------------------------------------------------
    auto qos = rclcpp::QoS(10);

    lidar_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/lidar", qos);
    imu_pub_   = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/imu",   qos);
    gps_pub_   = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/gps",   qos);
    all_pub_   = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/all",   qos);

    // ------------------------------------------------------------------
    // Periodic publish timer
    // ------------------------------------------------------------------
    auto period = std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SensorHealthMonitor::publish_health, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "sensor_health_monitor started.\n"
      "  Publishing:\n"
      "    /sensor_health/lidar  (timeout: %.1fs)\n"
      "    /sensor_health/imu    (timeout: %.1fs)\n"
      "    /sensor_health/gps    (timeout: %.1fs)\n"
      "    /sensor_health/all\n"
      "  Publish rate: %.1f Hz",
      TIMEOUT_LIDAR_SEC, TIMEOUT_IMU_SEC, TIMEOUT_GPS_SEC, PUBLISH_RATE_HZ);
  }

private:
  // ------------------------------------------------------------------
  // publish_health — runs at PUBLISH_RATE_HZ
  // ------------------------------------------------------------------
  void publish_health()
  {
    auto now = this->now();

    bool lidar_ok = lidar_state_.is_healthy(now);
    bool imu_ok   = imu_state_.is_healthy(now);
    bool gps_ok   = gps_state_.is_healthy(now);
    bool all_ok   = lidar_ok && imu_ok && gps_ok;

    publish_bool(lidar_pub_, lidar_ok);
    publish_bool(imu_pub_,   imu_ok);
    publish_bool(gps_pub_,   gps_ok);
    publish_bool(all_pub_,   all_ok);

    log_status(lidar_ok, imu_ok, gps_ok);
  }

  // ------------------------------------------------------------------
  // publish_bool — tiny helper to avoid repeating msg construction
  // ------------------------------------------------------------------
  void publish_bool(
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & pub,
    bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    pub->publish(msg);
  }

  // ------------------------------------------------------------------
  // log_status — throttled terminal output
  // ------------------------------------------------------------------
  void log_status(bool lidar_ok, bool imu_ok, bool gps_ok)
  {
    // When all healthy: info log every 5 s so you know the node is alive
    if (lidar_ok && imu_ok && gps_ok) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[ALL HEALTHY] LiDAR=OK | IMU=OK | GPS=OK");
      return;
    }

    // When something is wrong: warn every 2 s per unhealthy sensor
    if (!lidar_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[UNHEALTHY] LiDAR — no data on %s within %.1fs",
        lidar_state_.topic.c_str(), lidar_state_.timeout_sec);
    }
    if (!imu_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[UNHEALTHY] IMU — no data on %s within %.1fs",
        imu_state_.topic.c_str(), imu_state_.timeout_sec);
    }
    if (!gps_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[UNHEALTHY] GPS — no data on %s within %.1fs",
        gps_state_.topic.c_str(), gps_state_.timeout_sec);
    }
  }

  // ------------------------------------------------------------------
  // Member variables
  // ------------------------------------------------------------------
  SensorState lidar_state_;
  SensorState imu_state_;
  SensorState gps_state_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr   gps_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gps_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr all_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};


// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
