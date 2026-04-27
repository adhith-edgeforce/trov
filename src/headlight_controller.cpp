#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>

class HeadlightController : public rclcpp::Node
{
public:
  HeadlightController() : Node("headlight_controller")
  {
    client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/trov/headlight",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        this->sendCommand(msg->data);
      });

    RCLCPP_INFO(this->get_logger(),
      "HeadlightController ready. Publish true/false to /trov/headlight");
  }

private:
  void sendCommand(bool turn_on)
  {
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "MAVRos command service not available");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->broadcast    = false;
    req->command      = 183;   // MAV_CMD_DO_SET_SERVO
    req->confirmation = 0;
    req->param1       = 9.0f;
    req->param2       = turn_on ? 2000.0f : 1000.0f;
    req->param3       = 0.0f;
    req->param4       = 0.0f;
    req->param5       = 0.0f;
    req->param6       = 0.0f;
    req->param7       = 0.0f;

    client_->async_send_request(
      req,
      [this, turn_on](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(this->get_logger(), "Headlight %s", turn_on ? "ON" : "OFF");
        } else {
          RCLCPP_WARN(this->get_logger(),
            "Command rejected (result=%d)", res->result);
        }
      });
  }

  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadlightController>());
  rclcpp::shutdown();
  return 0;
}