#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <algorithm>
#include <cmath>
#include <string>

class TrovBridgeDrive : public rclcpp::Node {
public:
  TrovBridgeDrive() : Node("trov_drive_bridge") {
    // ============ OUTPUT RANGE PARAMETERS (No GPIO pins - uses MAVROS) ============
    // These ±values are sent to Pixhawk via /mavros/manual_control/send
    // Pixhawk converts these ±values → PWM (1100-1900 µs) on its servo/motor ports
    // Configure actual PWM pins in Mission Planner: Setup → Servo Output
    // ============================================================================
    
    // Declare parameters
    // min_output (MAVRSS output range start): ±126 means output values range ±126 to ±500
    this->declare_parameter<double>("min_output", 126.0);
    // max_output (MAVROS output range end): ±500 for full motor authority
    this->declare_parameter<double>("max_output", 500.0);
    this->declare_parameter<double>("max_speed", 1.5);
    this->declare_parameter<double>("deadzone_velocity", 0.05);
    this->declare_parameter<double>("max_angular_rate", 1.5);
    this->declare_parameter<bool>("debug_logging", true);

    // Get parameters
    min_output_ = this->get_parameter("min_output").as_double();
    max_output_ = this->get_parameter("max_output").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    deadzone_vel_ = this->get_parameter("deadzone_velocity").as_double();
    max_angular_ = this->get_parameter("max_angular_rate").as_double();
    debug_ = this->get_parameter("debug_logging").as_bool();

    // Calculate scaling factor for linear velocity
    scale_ = (max_output_ - min_output_) / max_speed_;
    angular_scale_ = (max_output_ - min_output_) / max_angular_;

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", 10,
      std::bind(&TrovBridgeDrive::callback, this, std::placeholders::_1));

    pub_ = create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);

    // Debug publisher: publish PWM values for verification
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/trov/pwm_debug", 10);

    // Parameter callback to allow dynamic retuning
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&TrovBridgeDrive::parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "✅ TROV Drive Bridge initialized");
    RCLCPP_INFO(get_logger(), "   Subscribing to: /cmd_vel_smoothed");
    RCLCPP_INFO(get_logger(), "   Output Range: ±%.0f to ±%.0f",
      min_output_, max_output_);
    RCLCPP_INFO(get_logger(), "   Max Speed: %.2f m/s | Deadzone: %.3f m/s",
      max_speed_, deadzone_vel_);
    RCLCPP_INFO(get_logger(), "   Debug Logging: %s | Publishing to /trov/pwm_debug",
      debug_ ? "ENABLED" : "DISABLED");
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    mavros_msgs::msg::ManualControl mc;
    mc.header.stamp = now();

    double forward = msg->linear.x;
    double turn = msg->angular.z;

    // ============ OUTPUT VALUE MAPPING (±format for MAVROS ManualControl) ============
    // Output values: ±min_output to ±max_output (e.g., ±126 to ±500)
    // Positive values = forward/left | Negative values = reverse/right
    // Pixhawk FC converts these to PWM on pins configured in Mission Planner
    // Example: ±500 → 1100 PWM (reverse max) or 1900 PWM (forward max)
    // ============================================================================
    
    // Map velocity to ±value format with deadzone
    auto map_with_deadzone = [this](double input) -> int16_t {
      // Apply deadzone
      if (std::fabs(input) < deadzone_vel_)
        return 0;

      // Determine sign
      int16_t sign = (input > 0) ? 1 : -1;
      double mag = std::min(std::fabs(input), max_speed_);
      double out = min_output_ + mag * scale_;
      out = std::min(out, max_output_);
      return static_cast<int16_t>(sign * out);
    };

    auto map_angular = [this](double input) -> int16_t {
      if (std::fabs(input) < 0.01)
        return 0;

      int16_t sign = (input > 0) ? 1 : -1;
      double mag = std::min(std::fabs(input), max_angular_);
      double out = min_output_ + mag * angular_scale_;
      out = std::min(out, max_output_);
      return static_cast<int16_t>(sign * out);
    };

    mc.z = map_with_deadzone(forward);  // thrust: ±values → Pixhawk converts to PWM for motor/ESC
    mc.y = map_angular(-turn);  // steering: ±values → Pixhawk converts to PWM for servo
    mc.x = 0;  // not used in our setup
    mc.r = 0;  // not used in our setup

    if (debug_) {
      RCLCPP_DEBUG(this->get_logger(),
        "Input: fwd=%.3f ang=%.3f | PWM: thrust=%d steer=%d",
        forward, turn, mc.z, mc.y);
    }

    // Publish debug info
    std_msgs::msg::Float32MultiArray debug_msg;
    debug_msg.data.push_back(static_cast<float>(forward));
    debug_msg.data.push_back(static_cast<float>(turn));
    debug_msg.data.push_back(static_cast<float>(mc.z));
    debug_msg.data.push_back(static_cast<float>(mc.y));
    debug_pub_->publish(debug_msg);

    pub_->publish(mc);
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated";

    for (const auto &param : parameters) {
      if (param.get_name() == "min_output") {
        // min_output: Minimum magnitude of output values (±126-500 range start)
        // Controls motor responsiveness - lower = more degrees of freedom at low speeds
        min_output_ = param.as_double();
        scale_ = (max_output_ - min_output_) / max_speed_;
        RCLCPP_INFO(get_logger(), "Updated min_output (output range start): %.0f", min_output_);
      } else if (param.get_name() == "max_output") {
        // max_output: Maximum magnitude of output values (±126-500 range end)
        // Controls maximum motor command - higher = full authority to motors
        max_output_ = param.as_double();
        scale_ = (max_output_ - min_output_) / max_speed_;
        RCLCPP_INFO(get_logger(), "Updated max_output (output range end): %.0f", max_output_);
      } else if (param.get_name() == "max_speed") {
        max_speed_ = param.as_double();
        scale_ = (max_output_ - min_output_) / max_speed_;
        RCLCPP_INFO(get_logger(), "Updated max_speed: %.2f m/s", max_speed_);
      } else if (param.get_name() == "deadzone_velocity") {
        deadzone_vel_ = param.as_double();
        RCLCPP_INFO(get_logger(), "Updated deadzone_velocity: %.3f m/s", deadzone_vel_);
      } else if (param.get_name() == "debug_logging") {
        debug_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "Updated debug_logging: %s", debug_ ? "ON" : "OFF");
      }
    }
    return result;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr pub_;  // Sends ±values to Pixhawk
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;  // For monitoring
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ============ PARAMETER EXPLANATION ============
  // min_output, max_output: Define ±output range sent to Pixhawk
  //   e.g., min=126, max=500 means outputs range ±126 to ±500
  // Pixhawk FC then converts these ±values to PWM (1100-1900 µs)
  // No GPIO pins here - Pixhawk handles PWM generation on its servo outputs
  // Configure which FC pin controls which motor in Mission Planner
  // =============================================
  
  double min_output_;  // Start of output range magnitude
  double max_output_;  // End of output range magnitude
  double max_speed_;
  double deadzone_vel_;
  double max_angular_;
  double scale_;  // Linear scaling: (max_output - min_output) / max_speed
  double angular_scale_;  // Angular scaling: (max_output - min_output) / max_angular
  bool debug_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrovBridgeDrive>());
  rclcpp::shutdown();
  return 0;
}