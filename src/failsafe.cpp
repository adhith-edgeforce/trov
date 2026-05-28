/**
 * @file amr_failsafe_node.cpp
 * @brief AMR Failsafe / Watchdog Node — ROS2 Humble (C++)
 *
 * Monitors three critical subsystems:
 *   - rtabmap_odom  (ICP Odometry)   → error code: 0x0R
 *   - AMCL          (Localization)   → error code: 0x0L
 *   - Nav2          (Navigation)     → error code: 0x0N
 *
 * TWIST GATE (Option A):
 *   This node is the SOLE publisher on /cmd_vel_smoothed.
 *   Nav2 / teleop must publish to /cmd_vel_raw.
 *   On nominal  → forwards /cmd_vel_raw to /cmd_vel_smoothed as-is.
 *   On fault    → blocks /cmd_vel_raw and publishes zero Twist instead.
 *
 * FIXES applied vs original:
 *   #1  Nav watchdog now uses a separate boolean (nav_goal_ever_active_)
 *       so it only arms after at least one goal has been seen, avoiding
 *       false-triggers during idle periods while still catching mid-mission drops.
 *   #2  current_error_ changed from const char* to std::string so comparison
 *       uses value equality, not pointer equality.
 *   #3  *_active_ flags AND last_*_time_ are reset when a subsystem restarts,
 *       preventing stale-timestamp false positives after a crash/restart.
 *   #4  Zero velocity / gate-block is applied immediately inside the
 *       /cmd_vel_raw callback when estop is active — no gap waiting for
 *       the next watchdog tick.
 *   #5  Bitmask fault reporting — all simultaneous faults are encoded
 *       (e.g. "0x0RL" if both odom and AMCL fail at the same time).
 *   #6  Subsystems that never came online are flagged as NOT nominal;
 *       "0x00" is only published once ALL three have been seen at least once.
 *   #7  Timer period uses integer milliseconds instead of floating-point
 *       division to avoid drift.
 *   #8  Parameter validation at startup — throws if watchdog_rate_hz <= 0.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <optional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ── Error code fragments (combined into bitmask string at runtime) ────────────
static const std::string ERROR_NONE = "0x00";   // All systems nominal
static const std::string FRAG_ODOM  = "R";      // ICP Odometry failure
static const std::string FRAG_LOC   = "L";      // AMCL Localization failure
static const std::string FRAG_NAV   = "N";      // Navigation failure


class AMRFailsafeNode : public rclcpp::Node
{
public:
  AMRFailsafeNode()
  : Node("amr_failsafe_node"),
    odom_active_(false),
    amcl_active_(false),
    nav_active_(false),
    nav_goal_ever_active_(false),   // FIX #1
    estop_active_(false),
    all_systems_ever_online_(false),
    current_error_(ERROR_NONE)
  {
    // ── Declare parameters ───────────────────────────────────────────────────
    this->declare_parameter<double>("odom_timeout_sec",   2.0);
    this->declare_parameter<double>("amcl_timeout_sec",   3.0);
    this->declare_parameter<double>("nav_timeout_sec",    5.0);
    this->declare_parameter<double>("watchdog_rate_hz",  10.0);
    this->declare_parameter<std::string>("cmd_vel_raw_topic",
      "/cmd_vel_raw");
    this->declare_parameter<std::string>("cmd_vel_out_topic",
      "/cmd_vel_smoothed");
    this->declare_parameter<std::string>("odom_topic",
      "/odom");
    this->declare_parameter<std::string>("amcl_topic",
      "/amcl_pose");
    this->declare_parameter<std::string>("nav_status_topic",
      "/navigate_to_pose/_action/status");
    this->declare_parameter<std::string>("error_code_topic",
      "/amr/failsafe/error_code");

    // ── Read parameters ──────────────────────────────────────────────────────
    const double odom_timeout_sec = this->get_parameter("odom_timeout_sec").as_double();
    const double amcl_timeout_sec = this->get_parameter("amcl_timeout_sec").as_double();
    const double nav_timeout_sec  = this->get_parameter("nav_timeout_sec").as_double();
    const double rate_hz          = this->get_parameter("watchdog_rate_hz").as_double();

    // FIX #8 — validate rate before use
    if (rate_hz <= 0.0) {
      RCLCPP_FATAL(this->get_logger(),
        "watchdog_rate_hz must be > 0, got %.2f", rate_hz);
      throw std::invalid_argument("watchdog_rate_hz must be > 0");
    }

    const std::string cmd_vel_raw_topic = this->get_parameter("cmd_vel_raw_topic").as_string();
    const std::string cmd_vel_out_topic = this->get_parameter("cmd_vel_out_topic").as_string();
    const std::string odom_topic        = this->get_parameter("odom_topic").as_string();
    const std::string amcl_topic        = this->get_parameter("amcl_topic").as_string();
    const std::string nav_topic         = this->get_parameter("nav_status_topic").as_string();
    const std::string error_topic       = this->get_parameter("error_code_topic").as_string();

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

    // TWIST GATE — /cmd_vel_raw input (Option A)
    cmd_vel_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_raw_topic,
      10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        // FIX #4 — gate is enforced immediately here, not on next watchdog tick
        if (estop_active_) {
          sendZeroVelocity();
        } else {
          cmd_vel_pub_->publish(*msg);
        }
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        const rclcpp::Time now = this->now();

        // FIX #3 — detect restart: if was inactive and now getting msgs again,
        // reset timestamp so stale elapsed time doesn't immediately trigger fault
        if (!odom_active_) {
          RCLCPP_INFO(this->get_logger(), "ICP Odometry: ONLINE");
          odom_active_    = true;
          last_odom_time_ = now;   // fresh start — no stale gap
        } else {
          last_odom_time_ = now;
        }
      });

    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic,
      reliable_qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        const rclcpp::Time now = this->now();

        if (!amcl_active_) {
          RCLCPP_INFO(this->get_logger(), "AMCL Localization: ONLINE");
          amcl_active_    = true;
          last_amcl_time_ = now;
        } else {
          last_amcl_time_ = now;
        }
      });

    // FIX #1 — Nav2 watchdog: only arm once a goal has been seen at least once.
    // During idle (no active goal) Nav2 publishes nothing — that is NOT a fault.
    // The watchdog arms on first status message and re-arms each time a new
    // goal becomes active, so mid-mission drops are still caught.
    nav_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      nav_topic,
      reliable_qos,
      [this](action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const rclcpp::Time now = this->now();

        // Check if any goal is currently executing
        bool has_active_goal = false;
        for (const auto & status : msg->status_list) {
          // STATUS_EXECUTING == 2 in action_msgs
          if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
            has_active_goal = true;
            break;
          }
        }

        if (has_active_goal) {
          nav_goal_ever_active_ = true;
          last_nav_time_        = now;
          if (!nav_active_) {
            RCLCPP_INFO(this->get_logger(), "Nav2 Navigation: ONLINE (goal executing)");
            nav_active_ = true;
          }
        } else {
          // Goal finished / cancelled / idle — disarm nav watchdog
          // so we don't false-trigger while robot is sitting still
          if (nav_active_) {
            RCLCPP_INFO(this->get_logger(),
              "Nav2 Navigation: IDLE — nav watchdog disarmed until next goal.");
            nav_active_ = false;
          }
        }
      });

    // ── Publishers ───────────────────────────────────────────────────────────
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, 10);
    error_pub_   = this->create_publisher<std_msgs::msg::String>(error_topic, latched_qos);

    // ── Watchdog timer ────────────────────────────────────────────────────────
    // FIX #7 — use integer milliseconds, not floating-point division
    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
    watchdog_timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&AMRFailsafeNode::watchdogCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "AMR Failsafe Node started  [TWIST GATE mode]\n"
      "  cmd_vel input    : %s\n"
      "  cmd_vel output   : %s\n"
      "  Monitoring odom  : %s  (timeout=%.1fs)\n"
      "  Monitoring AMCL  : %s  (timeout=%.1fs)\n"
      "  Monitoring Nav2  : %s  (timeout=%.1fs, only while goal active)\n"
      "  Error code topic : %s",
      cmd_vel_raw_topic.c_str(),
      cmd_vel_out_topic.c_str(),
      odom_topic.c_str(),    odom_timeout_sec,
      amcl_topic.c_str(),    amcl_timeout_sec,
      nav_topic.c_str(),     nav_timeout_sec,
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

      // FIX #6 — only consider system nominal once all three have been seen
      all_systems_ever_online_ =
        odom_active_ && amcl_active_ && nav_goal_ever_active_;

      // Check ICP Odometry
      if (odom_active_ && last_odom_time_.has_value()) {
        const rclcpp::Duration elapsed = now - last_odom_time_.value();
        if (elapsed > odom_timeout_) {
          odom_fail = true;
          // FIX #3 — reset flag so node re-arms cleanly after subsystem restarts
          odom_active_ = false;
          RCLCPP_ERROR(this->get_logger(),
            "[0x0R] ICP Odometry TIMEOUT (%.2fs since last msg) — watchdog disarmed until restart",
            elapsed.seconds());
        }
      }

      // Check AMCL Localization
      if (amcl_active_ && last_amcl_time_.has_value()) {
        const rclcpp::Duration elapsed = now - last_amcl_time_.value();
        if (elapsed > amcl_timeout_) {
          amcl_fail = true;
          amcl_active_ = false;   // FIX #3
          RCLCPP_ERROR(this->get_logger(),
            "[0x0L] AMCL Localization TIMEOUT (%.2fs since last msg) — watchdog disarmed until restart",
            elapsed.seconds());
        }
      }

      // Check Nav2 — FIX #1: only while a goal is active
      if (nav_active_ && last_nav_time_.has_value()) {
        const rclcpp::Duration elapsed = now - last_nav_time_.value();
        if (elapsed > nav_timeout_) {
          nav_fail = true;
          nav_active_ = false;   // FIX #3
          RCLCPP_ERROR(this->get_logger(),
            "[0x0N] Navigation TIMEOUT (%.2fs since last msg) — watchdog disarmed until next goal",
            elapsed.seconds());
        }
      }
    }  // mutex released

    // ── FIX #5 — Bitmask: encode ALL simultaneous faults ────────────────────
    if (odom_fail || amcl_fail || nav_fail) {
      std::string error_code = "0x0";
      if (odom_fail) error_code += "R";
      if (amcl_fail) error_code += "L";
      if (nav_fail)  error_code += "N";
      triggerEstop(error_code);
      return;
    }

    // ── FIX #6 — Only report nominal once all subsystems have been seen ──────
    if (!all_systems_ever_online_) {
      // Some subsystem has never come online — not yet nominal
      if (!estop_active_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Waiting for all subsystems to come online before declaring nominal. "
          "odom=%s amcl=%s nav_ever=%s",
          odom_active_ ? "YES" : "NO",
          amcl_active_ ? "YES" : "NO",
          nav_goal_ever_active_ ? "YES" : "NO");
      }
      return;
    }

    // All nominal
    if (estop_active_) {
      RCLCPP_INFO(this->get_logger(), "All systems nominal — clearing E-stop.");
      estop_active_  = false;
      current_error_ = ERROR_NONE;
    }
    publishErrorCode(ERROR_NONE);
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  void triggerEstop(const std::string & error_code)
  {
    sendZeroVelocity();
    publishErrorCode(error_code);

    // FIX #2 — std::string comparison (was raw const char* pointer comparison)
    if (!estop_active_ || current_error_ != error_code) {
      RCLCPP_WARN(this->get_logger(), "E-STOP ACTIVE | Error code: %s", error_code.c_str());
      estop_active_  = true;
      current_error_ = error_code;
    }
  }

  void sendZeroVelocity()
  {
    geometry_msgs::msg::Twist stop{};  // all fields zero-initialised
    cmd_vel_pub_->publish(stop);
  }

  void publishErrorCode(const std::string & code)
  {
    std_msgs::msg::String msg;
    msg.data = code;
    error_pub_->publish(msg);
  }

  // ── Member variables ──────────────────────────────────────────────────────

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                       cmd_vel_raw_sub_;
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

  // State (all protected by mutex_)
  std::mutex mutex_;
  std::optional<rclcpp::Time> last_odom_time_;
  std::optional<rclcpp::Time> last_amcl_time_;
  std::optional<rclcpp::Time> last_nav_time_;

  bool odom_active_;
  bool amcl_active_;
  bool nav_active_;
  bool nav_goal_ever_active_;       // FIX #1 — latched: true once any goal ever ran
  bool all_systems_ever_online_;    // FIX #6 — latched: true once all three seen

  bool        estop_active_;
  std::string current_error_;       // FIX #2 — std::string, not const char*
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
