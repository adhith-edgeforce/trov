#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <cstdlib>
#include <atomic>
#include <thread>

class BeepOnInputNode : public rclcpp::Node
{
public:
  BeepOnInputNode() : Node("beep_on_input_node"), is_playing_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("beep_sound_path", "/data/trov_ws/Beep.mp3");
    this->declare_parameter<int>("beep_volume", 20);

    beep_path_ = this->get_parameter("beep_sound_path").as_string();
    beep_volume_ = this->get_parameter("beep_volume").as_int();

    RCLCPP_INFO(this->get_logger(), "Beep sound path: %s", beep_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Beep volume: %d%%", beep_volume_);

    sub_ = this->create_subscription<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send",
      10,
      std::bind(&BeepOnInputNode::controlCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "BeepOnInput node started. Listening on /mavros/manual_control/send");
  }

private:
  void controlCallback(const mavros_msgs::msg::ManualControl::SharedPtr msg)
  {
    // Check if any axis is non-zero (x, y, z, r are typically -1000 to 1000)
    bool has_input = (msg->x != 0.0f || msg->y != 0.0f ||
                      msg->z != 0.0f || msg->r != 0.0f);

    if (has_input && !is_playing_.load())
    {
      RCLCPP_INFO(this->get_logger(),
        "Manual input detected [x=%.1f y=%.1f z=%.1f r=%.1f] — playing beep at %d%% volume",
        msg->x, msg->y, msg->z, msg->r, beep_volume_);
      playBeepAsync();
    }
  }

  void playBeepAsync()
  {
    is_playing_.store(true);

    std::thread([this]() {
      // Convert 0-100 integer to 0.0-1.0 float for sox vol
      float vol_fraction = beep_volume_ / 100.0f;
      std::string cmd = "play -q " + beep_path_ +
                        " vol " + std::to_string(vol_fraction) +
                        " > /dev/null 2>&1";
      std::system(cmd.c_str());
      is_playing_.store(false);
    }).detach();
  }

  rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr sub_;
  std::string beep_path_;
  int beep_volume_;
  std::atomic<bool> is_playing_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeepOnInputNode>());
  rclcpp::shutdown();
  return 0;
}