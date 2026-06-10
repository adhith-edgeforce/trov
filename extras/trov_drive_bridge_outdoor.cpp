#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

enum class DriveState { STRAIGHT, SLOWING, STOPPING, PIVOT, STALLED };

class TrovBridgeDrive : public rclcpp::Node {
public:
  TrovBridgeDrive() : Node("trov_drive_bridge") {
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed", 10,
      std::bind(&TrovBridgeDrive::callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrovBridgeDrive::odom_callback, this, std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&TrovBridgeDrive::state_callback, this, std::placeholders::_1));

    pub_ = create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);

    RCLCPP_INFO(get_logger(),
      "TROV Drive Bridge started — max_speed=%.2f max_output=%.0f "
      "pivot_output_min=%.0f pivot_output_max=%.0f pivot_output_step=%.0f "
      "ang_threshold=%.3f ang_slowzone=%.3f stop_duration_ms=%d "
      "stall_check_ms=%d stall_dist=%.4f stall_zero_ms=%d",
      MAX_SPEED_, MAX_OUTPUT_,
      PIVOT_OUTPUT_MIN_, PIVOT_OUTPUT_MAX_, PIVOT_OUTPUT_STEP_,
      ANG_THRESHOLD_, ANG_SLOW_ZONE_,
      STOP_DURATION_MS_, STALL_CHECK_MS_, STALL_DIST_THRESHOLD_,
      STALL_ZERO_MS_);
  }

private:
  // ── Drive constants ────────────────────────────────────────────────────
  static constexpr double MAX_SPEED_        = 1.5;
  static constexpr double MAX_OUTPUT_       = 450.0;
  static constexpr double MIN_OUTPUT_       = 0.0;
  static constexpr double DEADZONE_         = 0.09;
  static constexpr double ANG_THRESHOLD_    = 0.3;
  static constexpr double ANG_SLOW_ZONE_    = 0.2;
  static constexpr double SLOW_OUTPUT_      = 150.0;
  static constexpr int    STOP_DURATION_MS_ = 2000;

  // ── Adaptive pivot constants ───────────────────────────────────────────
  //static constexpr double PIVOT_OUTPUT_MIN_    = 450.0; 
  static constexpr double PIVOT_OUTPUT_MIN_    = 475.0;  // 350.0 for indoor
  static constexpr double PIVOT_OUTPUT_MAX_    = 900.0;
  static constexpr double PIVOT_OUTPUT_STEP_   = 75.0; //75 for indoor
  static constexpr int    PIVOT_CHECK_MS_      = 1500;
  static constexpr double PIVOT_ROT_THRESHOLD_ = 0.5;  // radians

  // ── Stall-detection constants ──────────────────────────────────────────
  static constexpr int    STALL_CHECK_MS_       = 7000;
  static constexpr double STALL_DIST_THRESHOLD_ = 0.02;
  static constexpr int    STALL_ZERO_MS_        = 1000;

  // ── Runtime state ──────────────────────────────────────────────────────
  DriveState   state_    = DriveState::STRAIGHT;
  rclcpp::Time stop_start_;
  double       last_ang_ = 0.0;

  // Arm state
  bool is_armed_  = false;
  bool was_armed_ = false;

  // Stall tracking
  rclcpp::Time stall_window_start_;
  double       stall_window_x_    = 0.0;
  double       stall_window_y_    = 0.0;
  bool         stall_window_set_  = false;
  rclcpp::Time stall_zero_start_;
  bool         output_is_nonzero_ = false;

  // Adaptive pivot tracking
  double       current_pivot_output_ = PIVOT_OUTPUT_MIN_;
  rclcpp::Time pivot_check_start_;
  double       pivot_start_yaw_      = 0.0;
  bool         pivot_check_set_      = false;

  // Latest odom
  double odom_x_   = 0.0;
  double odom_y_   = 0.0;
  double odom_yaw_ = 0.0;
  bool   odom_ok_  = false;

  // ── Arm state callback ─────────────────────────────────────────────────
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    is_armed_ = msg->armed;

    // Edge detection: armed → disarmed
    if (was_armed_ && !is_armed_) {
      RCLCPP_WARN(get_logger(),
        "DISARM detected — resetting pivot output and stall window.");
      reset_pivot_state();
      reset_stall_window();
      // Push state machine back to STRAIGHT so next arm starts clean
      state_ = DriveState::STRAIGHT;
    }

    was_armed_ = is_armed_;
  }

  // ── Odom callback ──────────────────────────────────────────────────────
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;

    odom_ok_ = true;
  }

  // ── Helpers ────────────────────────────────────────────────────────────
  void reset_pivot_state() {
    current_pivot_output_ = PIVOT_OUTPUT_MIN_;
    pivot_check_set_      = false;
  }

  void reset_stall_window() {
    stall_window_set_ = false;
  }

  // ── Drive callback ─────────────────────────────────────────────────────
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    mavros_msgs::msg::ManualControl mc;
    mc.header.stamp = now();
    mc.x = 0; mc.y = 0; mc.z = 0; mc.r = 0; mc.buttons = 0;

    // ── If disarmed, publish zeros and skip everything ────────────────────
    if (!is_armed_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Vehicle DISARMED — holding zero output.");
      pub_->publish(mc);
      output_is_nonzero_ = false;
      return;
    }

    const double lin = apply_deadzone(msg->linear.x);
    const double ang = apply_deadzone(msg->angular.z);
    const double throttle_scale = (MAX_OUTPUT_ - MIN_OUTPUT_) / MAX_SPEED_;

    // ── STALLED: hold zeros until timer expires, then resume ──────────────
    if (state_ == DriveState::STALLED) {
      const auto elapsed_ms = (now() - stall_zero_start_).nanoseconds() / 1'000'000;
      if (elapsed_ms < STALL_ZERO_MS_) {
        mc.z = 0; mc.y = 0;
        RCLCPP_INFO(get_logger(),
          "cmd_vel [lin=%.3f, ang=%.3f] -> STALLED  [z=0, y=0] (%ldms remaining)",
          msg->linear.x, msg->angular.z,
          static_cast<long>(STALL_ZERO_MS_ - elapsed_ms));
        pub_->publish(mc);
        output_is_nonzero_ = false;
        return;
      }
      RCLCPP_INFO(get_logger(), "-> STALLED complete, resuming normal control");
      state_ = DriveState::STRAIGHT;
      reset_stall_window();
    }

    // ── Normal state transitions ──────────────────────────────────────────
    if (state_ != DriveState::STOPPING) {
      if (std::fabs(ang) > ANG_THRESHOLD_) {
        if (state_ != DriveState::PIVOT) {
          state_      = DriveState::STOPPING;
          stop_start_ = now();
          last_ang_   = ang;
          reset_pivot_state();
          RCLCPP_INFO(get_logger(), "-> entering STOPPING phase (%dms)", STOP_DURATION_MS_);
        }
      } else if (std::fabs(ang) > ANG_SLOW_ZONE_) {
        if (state_ == DriveState::PIVOT) reset_pivot_state();
        state_ = DriveState::SLOWING;
      } else {
        if (state_ == DriveState::PIVOT) reset_pivot_state();
        state_ = DriveState::STRAIGHT;
      }
    } else {
      last_ang_ = ang;
      const auto elapsed_ms = (now() - stop_start_).nanoseconds() / 1'000'000;
      if (elapsed_ms >= STOP_DURATION_MS_) {
        state_ = DriveState::PIVOT;
        RCLCPP_INFO(get_logger(), "-> STOPPING complete, entering PIVOT");
      }
    }

    // ── Output ────────────────────────────────────────────────────────────
    switch (state_) {
      case DriveState::STRAIGHT:
        mc.z = map_linear_axis(lin, throttle_scale);
        mc.y = 0;
        RCLCPP_INFO(get_logger(),
          "cmd_vel [lin=%.3f, ang=%.3f] -> STRAIGHT [z=%f, y=%f]",
          msg->linear.x, msg->angular.z, mc.z, mc.y);
        break;

      case DriveState::SLOWING:
        mc.z = static_cast<int16_t>(SLOW_OUTPUT_);
        mc.y = 0;
        RCLCPP_INFO(get_logger(),
          "cmd_vel [lin=%.3f, ang=%.3f] -> SLOWING  [z=%f, y=%f]",
          msg->linear.x, msg->angular.z, mc.z, mc.y);
        break;

      case DriveState::STOPPING:
        mc.z = 0; mc.y = 0;
        RCLCPP_INFO(get_logger(),
          "cmd_vel [lin=%.3f, ang=%.3f] -> STOPPING [z=%f, y=%f]",
          msg->linear.x, msg->angular.z, mc.z, mc.y);
        break;

      case DriveState::PIVOT: {
        mc.z = 0;

        // ── Adaptive pivot — ramp only when armed and odom is valid ──────
        if (odom_ok_) {
          if (!pivot_check_set_) {
            pivot_check_start_ = now();
            pivot_start_yaw_   = odom_yaw_;
            pivot_check_set_   = true;
          } else {
            const auto window_ms =
              (now() - pivot_check_start_).nanoseconds() / 1'000'000;

            if (window_ms >= PIVOT_CHECK_MS_) {
              double yaw_delta = std::fabs(odom_yaw_ - pivot_start_yaw_);
              if (yaw_delta > M_PI) yaw_delta = 2.0 * M_PI - yaw_delta;

              if (yaw_delta < PIVOT_ROT_THRESHOLD_) {
                current_pivot_output_ = std::min(
                  current_pivot_output_ + PIVOT_OUTPUT_STEP_,
                  PIVOT_OUTPUT_MAX_);
                RCLCPP_WARN(get_logger(),
                  "PIVOT: insufficient rotation (yaw_delta=%.3f rad in %ldms), "
                  "ramping output to %.0f",
                  yaw_delta, static_cast<long>(window_ms), current_pivot_output_);
              } else {
                RCLCPP_INFO(get_logger(),
                  "PIVOT: rotating OK (yaw_delta=%.3f rad), output stays at %.0f",
                  yaw_delta, current_pivot_output_);
              }

              // Slide window forward
              pivot_check_start_ = now();
              pivot_start_yaw_   = odom_yaw_;
            }
          }
        }

        mc.y = (last_ang_ > 0.0)
                 ? -static_cast<int16_t>(current_pivot_output_)
                 :  static_cast<int16_t>(current_pivot_output_);

        RCLCPP_INFO(get_logger(),
          "cmd_vel [lin=%.3f, ang=%.3f] -> PIVOT    [z=%f, y=%f] (pivot_output=%.0f)",
          msg->linear.x, msg->angular.z, mc.z, mc.y, current_pivot_output_);
        break;
      }

      case DriveState::STALLED:
        break; // handled above
    }

    output_is_nonzero_ = (mc.z != 0 || mc.y != 0);

    // ── Stall detection ───────────────────────────────────────────────────
    // is_armed_ is guaranteed true here (early return above handles disarmed)
    const bool in_moving_state =
      (state_ == DriveState::STRAIGHT ||
       state_ == DriveState::SLOWING  ||
       state_ == DriveState::PIVOT);

    if (in_moving_state && output_is_nonzero_ && odom_ok_) {
      if (!stall_window_set_) {
        stall_window_start_ = now();
        stall_window_x_     = odom_x_;
        stall_window_y_     = odom_y_;
        stall_window_set_   = true;
      } else {
        const auto window_ms =
          (now() - stall_window_start_).nanoseconds() / 1'000'000;

        if (window_ms >= STALL_CHECK_MS_) {
          const double dx   = odom_x_ - stall_window_x_;
          const double dy   = odom_y_ - stall_window_y_;
          const double dist = std::sqrt(dx * dx + dy * dy);

          if (dist < STALL_DIST_THRESHOLD_) {
            RCLCPP_WARN(get_logger(),
              "STALL DETECTED: moved only %.4fm in %ldms while commanding motion. "
              "Zeroing output for %dms.",
              dist, static_cast<long>(window_ms), STALL_ZERO_MS_);
            state_            = DriveState::STALLED;
            stall_zero_start_ = now();
            mc.z = 0; mc.y = 0;
            reset_stall_window();
            reset_pivot_state();
          } else {
            reset_stall_window();
          }
        }
      }
    } else {
      reset_stall_window();
    }

    pub_->publish(mc);
  }

  double apply_deadzone(double v) const {
    return (std::fabs(v) < DEADZONE_) ? 0.0 : v;
  }

  int16_t map_linear_axis(double input, double scale_val) const {
    if (std::fabs(input) < DEADZONE_) return 0;
    const int16_t sign = (input > 0.0) ? 1 : -1;
    const double  mag  = std::min(std::fabs(input), MAX_SPEED_);
    const double  out  = std::min(MIN_OUTPUT_ + mag * scale_val, MAX_OUTPUT_);
    return static_cast<int16_t>(sign * out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr      state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrovBridgeDrive>());
  rclcpp::shutdown();
  return 0;
}



