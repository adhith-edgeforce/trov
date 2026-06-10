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
 * Log behaviour:
 *   - On state change (healthy→unhealthy or back): logs immediately
 *   - While unhealthy: reminder log every 30s (not every 0.5s)
 *   - While all healthy: info log every 60s so you know node is alive
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
static constexpr double TIMEOUT_LIDAR_SEC  = 1.0;
static constexpr double TIMEOUT_IMU_SEC    = 0.5;
static constexpr double TIMEOUT_GPS_SEC    = 1.0;
static constexpr double PUBLISH_RATE_HZ    = 2.0;   // topic publish rate — unchanged

// Log throttle intervals (milliseconds)
static constexpr int    LOG_UNHEALTHY_MS   = 30000;  // reminder when staying unhealthy: every 30s
static constexpr int    LOG_ALL_HEALTHY_MS = 60000;  // heartbeat when all OK: every 60s
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// SensorState
// ---------------------------------------------------------------------------
struct SensorState
{
  std::string display_name;
  std::string topic;
  double      timeout_sec;

  bool          ever_received{false};
  rclcpp::Time  last_received;
  bool          last_healthy{true};   // tracks previous state for change detection

  void on_message(const rclcpp::Time & now)
  {
    last_received = now;
    ever_received = true;
  }

  bool is_healthy(const rclcpp::Time & now) const
  {
    if (!ever_received) return false;
    return (now - last_received).seconds() <= timeout_sec;
  }
};


// ---------------------------------------------------------------------------
// SensorHealthMonitor
// ---------------------------------------------------------------------------
class SensorHealthMonitor : public rclcpp::Node
{
public:
  SensorHealthMonitor()
  : Node("sensor_health_monitor")
  {
    lidar_state_.display_name = "LiDAR";
    lidar_state_.topic        = "/points";
    lidar_state_.timeout_sec  = TIMEOUT_LIDAR_SEC;

    imu_state_.display_name   = "IMU";
    imu_state_.topic          = "/imu/data";
    imu_state_.timeout_sec    = TIMEOUT_IMU_SEC;

    gps_state_.display_name   = "GPS";
    gps_state_.topic          = "/mavros/global_position/global";
    gps_state_.timeout_sec    = TIMEOUT_GPS_SEC;

    auto sensor_qos = rclcpp::SensorDataQoS();

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr) {
        lidar_state_.on_message(this->now());
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr) {
        imu_state_.on_message(this->now());
      });

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::QoS(10),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr) {
        gps_state_.on_message(this->now());
      });

    auto qos = rclcpp::QoS(10);
    lidar_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/lidar", qos);
    imu_pub_   = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/imu",   qos);
    gps_pub_   = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/gps",   qos);
    all_pub_   = this->create_publisher<std_msgs::msg::Bool>("/sensor_health/all",   qos);

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
      "  Publish rate: %.1f Hz\n"
      "  Log on state change + reminder every %ds if unhealthy",
      TIMEOUT_LIDAR_SEC, TIMEOUT_IMU_SEC, TIMEOUT_GPS_SEC,
      PUBLISH_RATE_HZ, LOG_UNHEALTHY_MS / 1000);
  }

private:
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

    log_status(lidar_ok, imu_ok, gps_ok, all_ok);

    // Update previous state for next tick
    lidar_state_.last_healthy = lidar_ok;
    imu_state_.last_healthy   = imu_ok;
    gps_state_.last_healthy   = gps_ok;
  }

  void publish_bool(
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & pub,
    bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    pub->publish(msg);
  }

  // ── Log only on state change + throttled reminders ──────────────────────
  void log_status(bool lidar_ok, bool imu_ok, bool gps_ok, bool all_ok)
  {
    // ── State-change logs — immediate, no throttle ────────────────────────
    log_sensor_change(lidar_state_, lidar_ok, "LiDAR");
    log_sensor_change(imu_state_,   imu_ok,   "IMU");
    log_sensor_change(gps_state_,   gps_ok,   "GPS");

    // ── Throttled reminder while something stays unhealthy ────────────────
    if (!lidar_ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), LOG_UNHEALTHY_MS,
        "[UNHEALTHY] LiDAR — no data on %s within %.1fs",
        lidar_state_.topic.c_str(), lidar_state_.timeout_sec);
    }
    if (!imu_ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), LOG_UNHEALTHY_MS,
        "[UNHEALTHY] IMU — no data on %s within %.1fs",
        imu_state_.topic.c_str(), imu_state_.timeout_sec);
    }
    if (!gps_ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), LOG_UNHEALTHY_MS,
        "[UNHEALTHY] GPS — no data on %s within %.1fs",
        gps_state_.topic.c_str(), gps_state_.timeout_sec);
    }

    // ── Heartbeat when all healthy ────────────────────────────────────────
    if (all_ok) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_ALL_HEALTHY_MS,
        "[ALL HEALTHY] LiDAR=OK | IMU=OK | GPS=OK");
    }
  }

  // Log immediately on transition (healthy→unhealthy or unhealthy→healthy)
  void log_sensor_change(const SensorState & state, bool now_healthy, const char * name)
  {
    if (now_healthy == state.last_healthy) return;  // no change, skip

    if (!now_healthy) {
      RCLCPP_WARN(this->get_logger(),
        "[SENSOR LOST] %s — no data on %s within %.1fs",
        name, state.topic.c_str(), state.timeout_sec);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "[SENSOR RECOVERED] %s is healthy again", name);
    }
  }

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