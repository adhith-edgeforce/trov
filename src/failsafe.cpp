/**
 * @file amr_failsafe_node.cpp
 * @brief AMR Failsafe / Watchdog Node — ROS2 Humble (C++)
 *
 * Monitors three critical subsystems:
 *   - rtabmap_odom  (ICP Odometry)   → error code: 0x0R
 *   - AMCL          (Localization)   → error code: 0x0L
 *   - Nav2          (Navigation)     → error code: 0x0N
 *
 * On failure the node:
 *   1. Publishes a zero Twist to cmd_vel_smoothed every watchdog cycle.
 *   2. Publishes the error code string to /amr/failsafe/error_code.
 *
 * Recovery is automatic: once all subsystems resume publishing the
 * E-stop is cleared and "0x00" (nominal) is published.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ── Error code constants ───────────────────────────────────────────────────────
static constexpr const char * ERROR_NONE = "0x00";  // All systems nominal
static constexpr const char * ERROR_ODOM = "0x0R";  // ICP Odometry failure
static constexpr const char * ERROR_LOC  = "0x0L";  // AMCL Localization failure
static constexpr const char * ERROR_NAV  = "0x0N";  // Navigation failure


class AMRFailsafeNode : public rclcpp::Node
{
public:
  AMRFailsafeNode()
  : Node("amr_failsafe_node"),
    odom_active_(false),
    amcl_active_(false),
    nav_active_(false),
    estop_active_(false),
    current_error_(ERROR_NONE)
  {
    // ── Declare parameters ───────────────────────────────────────────────────
    this->declare_parameter<double>("odom_timeout_sec",  2.0);
    this->declare_parameter<double>("amcl_timeout_sec",  3.0);
    this->declare_parameter<double>("nav_timeout_sec",   5.0);
    this->declare_parameter<double>("watchdog_rate_hz", 10.0);
    this->declare_parameter<std::string>("cmd_vel_topic",    "/cmd_vel_smoothed");
    this->declare_parameter<std::string>("odom_topic",       "/odom");
    this->declare_parameter<std::string>("amcl_topic",       "/amcl_pose");
    this->declare_parameter<std::string>("nav_status_topic",
      "/navigate_to_pose/_action/status");
    this->declare_parameter<std::string>("error_code_topic", "/amr/failsafe/error_code");

    // ── Read parameters ──────────────────────────────────────────────────────
    const double odom_timeout_sec = this->get_parameter("odom_timeout_sec").as_double();
    const double amcl_timeout_sec = this->get_parameter("amcl_timeout_sec").as_double();
    const double nav_timeout_sec  = this->get_parameter("nav_timeout_sec").as_double();
    const double rate_hz          = this->get_parameter("watchdog_rate_hz").as_double();

    const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    const std::string odom_topic    = this->get_parameter("odom_topic").as_string();
    const std::string amcl_topic    = this->get_parameter("amcl_topic").as_string();
    const std::string nav_topic     = this->get_parameter("nav_status_topic").as_string();
    const std::string error_topic   = this->get_parameter("error_code_topic").as_string();

    // Convert seconds → nanoseconds for rclcpp::Duration comparison
    odom_timeout_ = rclcpp::Duration::from_seconds(odom_timeout_sec);
    amcl_timeout_ = rclcpp::Duration::from_seconds(amcl_timeout_sec);
    nav_timeout_  = rclcpp::Duration::from_seconds(nav_timeout_sec);

    // ── QoS profiles ─────────────────────────────────────────────────────────
    rclcpp::QoS sensor_qos(10);
    sensor_qos.best_effort();

    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable();

    rclcpp::QoS latched_qos(1);
    latched_qos.reliable().transient_local();

    // ── Subscribers ──────────────────────────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_odom_time_ = this->now();
        if (!odom_active_) {
          RCLCPP_INFO(this->get_logger(), "ICP Odometry: ONLINE");
          odom_active_ = true;
        }
      });

    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic,
      reliable_qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_amcl_time_ = this->now();
        if (!amcl_active_) {
          RCLCPP_INFO(this->get_logger(), "AMCL Localization: ONLINE");
          amcl_active_ = true;
        }
      });

    nav_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      nav_topic,
      reliable_qos,
      [this](action_msgs::msg::GoalStatusArray::SharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_time_ = this->now();
        if (!nav_active_) {
          RCLCPP_INFO(this->get_logger(), "Nav2 Navigation: ONLINE");
          nav_active_ = true;
        }
      });

    // ── Publishers ───────────────────────────────────────────────────────────
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    error_pub_   = this->create_publisher<std_msgs::msg::String>(error_topic, latched_qos);

    // ── Watchdog timer ───────────────────────────────────────────────────────
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AMRFailsafeNode::watchdogCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "AMR Failsafe Node started.\n"
      "  Monitoring odom  : %s  (timeout=%.1fs)\n"
      "  Monitoring AMCL  : %s  (timeout=%.1fs)\n"
      "  Monitoring Nav2  : %s  (timeout=%.1fs)\n"
      "  E-stop topic     : %s\n"
      "  Error code topic : %s",
      odom_topic.c_str(),    odom_timeout_sec,
      amcl_topic.c_str(),    amcl_timeout_sec,
      nav_topic.c_str(),     nav_timeout_sec,
      cmd_vel_topic.c_str(),
      error_topic.c_str());
  }

private:
  // ── Watchdog callback ──────────────────────────────────────────────────────
  void watchdogCallback()
  {
    const rclcpp::Time now = this->now();

    bool odom_fail = false;
    bool amcl_fail = false;
    bool nav_fail  = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      // Check ICP Odometry
      if (odom_active_ && last_odom_time_.has_value()) {
        const rclcpp::Duration elapsed = now - last_odom_time_.value();
        if (elapsed > odom_timeout_) {
          odom_fail = true;
          RCLCPP_ERROR(this->get_logger(),
            "[%s] ICP Odometry TIMEOUT (%.2fs since last msg)",
            ERROR_ODOM, elapsed.seconds());
        }
      }

      // Check AMCL Localization
      if (amcl_active_ && last_amcl_time_.has_value()) {
        const rclcpp::Duration elapsed = now - last_amcl_time_.value();
        if (elapsed > amcl_timeout_) {
          amcl_fail = true;
          RCLCPP_ERROR(this->get_logger(),
            "[%s] AMCL Localization TIMEOUT (%.2fs since last msg)",
            ERROR_LOC, elapsed.seconds());
        }
      }

      // Check Nav2
      if (nav_active_ && last_nav_time_.has_value()) {
        const rclcpp::Duration elapsed = now - last_nav_time_.value();
        if (elapsed > nav_timeout_) {
          nav_fail = true;
          RCLCPP_ERROR(this->get_logger(),
            "[%s] Navigation TIMEOUT (%.2fs since last msg)",
            ERROR_NAV, elapsed.seconds());
        }
      }
    }  // mutex released

    // ── Act on faults (priority: ODOM > LOC > NAV) ───────────────────────────
    if (odom_fail) {
      triggerEstop(ERROR_ODOM);
    } else if (amcl_fail) {
      triggerEstop(ERROR_LOC);
    } else if (nav_fail) {
      triggerEstop(ERROR_NAV);
    } else {
      // All nominal
      if (estop_active_) {
        RCLCPP_INFO(this->get_logger(), "All systems nominal — clearing E-stop.");
        estop_active_  = false;
        current_error_ = ERROR_NONE;
      }
      publishErrorCode(ERROR_NONE);
    }
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  void triggerEstop(const char * error_code)
  {
    sendZeroVelocity();
    publishErrorCode(error_code);

    if (!estop_active_ || current_error_ != error_code) {
      RCLCPP_WARN(this->get_logger(), "E-STOP ACTIVE | Error code: %s", error_code);
      estop_active_  = true;
      current_error_ = error_code;
    }
  }

  void sendZeroVelocity()
  {
    geometry_msgs::msg::Twist stop{};  // all fields zero-initialised
    cmd_vel_pub_->publish(stop);
  }

  void publishErrorCode(const char * code)
  {
    std_msgs::msg::String msg;
    msg.data = code;
    error_pub_->publish(msg);
  }

  // ── Member variables ──────────────────────────────────────────────────────

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                         odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr   amcl_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr               nav_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       error_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Timeouts
  rclcpp::Duration odom_timeout_{0, 0};
  rclcpp::Duration amcl_timeout_{0, 0};
  rclcpp::Duration nav_timeout_{0, 0};

  // State (protected by mutex_)
  std::mutex mutex_;
  std::optional<rclcpp::Time> last_odom_time_;
  std::optional<rclcpp::Time> last_amcl_time_;
  std::optional<rclcpp::Time> last_nav_time_;

  bool odom_active_;
  bool amcl_active_;
  bool nav_active_;

  bool        estop_active_;
  const char * current_error_;
};


// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AMRFailsafeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}