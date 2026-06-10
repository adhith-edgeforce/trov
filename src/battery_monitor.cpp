#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <stdexcept>
#include <string>

// ── ADS1115 constants ────────────────────────────────────────────────────────
static constexpr int     I2C_BUS       = 7;
static constexpr uint8_t ADS_ADDR      = 0x48;
static constexpr uint8_t REG_CONVERT   = 0x00;
static constexpr uint8_t REG_CONFIG    = 0x01;

// Config bytes: single-shot, AIN0 vs GND, PGA ±2.048 V, 128 SPS
static constexpr uint8_t CFG_HI        = 0xC4;
static constexpr uint8_t CFG_LO        = 0x83;
static constexpr float   PGA_RANGE     = 2.048f;

// ── Battery calibration ──────────────────────────────────────────────────────
static constexpr float   V_MIN         = 1.37f;
static constexpr float   V_MAX         = 1.49f;

// ── Thresholds ───────────────────────────────────────────────────────────────
static constexpr float   THR_LOW       = 30.0f;
static constexpr float   THR_CRITICAL  = 15.0f;

// ── Publish rate ─────────────────────────────────────────────────────────────
static constexpr double  PUBLISH_HZ    = 1.0;

// ── Log every N callbacks (1 Hz publish → every 10s at N=10) ─────────────────
static constexpr int     LOG_EVERY_N   = 10;

class BatteryMonitorNode : public rclcpp::Node
{
public:
  BatteryMonitorNode()
  : Node("battery_monitor"), log_counter_(0), last_status_(""), last_percent_(0.0f)
  {
    // Open I2C bus
    std::string bus_path = "/dev/i2c-" + std::to_string(I2C_BUS);
    i2c_fd_ = open(bus_path.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s — check permissions (sudo usermod -aG i2c $USER)",
                   bus_path.c_str());
      throw std::runtime_error("I2C open failed");
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, ADS_ADDR) < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to set I2C slave address 0x%02X", ADS_ADDR);
      throw std::runtime_error("I2C ioctl failed");
    }

    RCLCPP_INFO(get_logger(), "ADS1115 opened on /dev/i2c-%d @ 0x%02X", I2C_BUS, ADS_ADDR);
    RCLCPP_INFO(get_logger(), "Calibration: V_MIN=%.3f V  V_MAX=%.3f V", V_MIN, V_MAX);
    RCLCPP_INFO(get_logger(), "Publishing at %.0f Hz, logging every %ds", PUBLISH_HZ, LOG_EVERY_N);

    // Publishers
    pub_percent_ = create_publisher<std_msgs::msg::Float32>("/trov/battery/percent", 10);
    pub_status_  = create_publisher<std_msgs::msg::String> ("/trov/battery/status",  10);

    // Timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / PUBLISH_HZ),
      std::bind(&BatteryMonitorNode::timer_cb, this));
  }

  ~BatteryMonitorNode()
  {
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
    }
  }

private:
  // ── Read one conversion from ADS1115 ───────────────────────────────────────
  float read_voltage()
  {
    uint8_t config[3] = { REG_CONFIG, CFG_HI, CFG_LO };
    if (write(i2c_fd_, config, 3) != 3) {
      throw std::runtime_error("I2C config write failed");
    }

    for (int attempt = 0; attempt < 20; ++attempt) {
      usleep(8000);

      uint8_t reg = REG_CONFIG;
      if (write(i2c_fd_, &reg, 1) != 1) {
        throw std::runtime_error("I2C register select failed");
      }
      uint8_t buf[2] = {};
      if (read(i2c_fd_, buf, 2) != 2) {
        throw std::runtime_error("I2C config read failed");
      }
      if (buf[0] & 0x80) {
        break;
      }
    }

    uint8_t reg = REG_CONVERT;
    if (write(i2c_fd_, &reg, 1) != 1) {
      throw std::runtime_error("I2C register select failed");
    }

    uint8_t raw[2] = {};
    if (read(i2c_fd_, raw, 2) != 2) {
      throw std::runtime_error("I2C conversion read failed");
    }

    int16_t value = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    return static_cast<float>(value) * PGA_RANGE / 32768.0f;
  }

  // ── Map voltage → percent ──────────────────────────────────────────────────
  static float voltage_to_percent(float voltage)
  {
    float pct = (voltage - V_MIN) / (V_MAX - V_MIN) * 100.0f;
    return std::max(0.0f, std::min(100.0f, pct));
  }

  // ── Timer callback ─────────────────────────────────────────────────────────
  void timer_cb()
  {
    float voltage = 0.0f;
    try {
      voltage = read_voltage();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ADC read error: %s", e.what());
      return;
    }

    float percent = voltage_to_percent(voltage);

    std::string status;
    if (percent < THR_CRITICAL) {
      status = "CRITICAL";
    } else if (percent < THR_LOW) {
      status = "LOW";
    } else {
      status = "OK";
    }

    // Publish every tick (keeps topic fresh for subscribers)
    auto msg_pct = std_msgs::msg::Float32();
    msg_pct.data = percent;
    pub_percent_->publish(msg_pct);

    auto msg_st = std_msgs::msg::String();
    msg_st.data = status;
    pub_status_->publish(msg_st);

    // ── Always log on status change (LOW / CRITICAL) ──────────────────────
    bool status_changed = (status != last_status_);
    bool is_warning     = (status == "LOW" || status == "CRITICAL");

    if (status_changed) {
      if (is_warning) {
        RCLCPP_WARN(get_logger(), "Battery status changed: [%s] %.1f%%  (%.4f V)",
                    status.c_str(), percent, voltage);
      } else {
        RCLCPP_INFO(get_logger(), "Battery status changed: [%s] %.1f%%  (%.4f V)",
                    status.c_str(), percent, voltage);
      }
      last_status_ = status;
    }

    // ── Periodic log every LOG_EVERY_N seconds ────────────────────────────
    log_counter_++;
    if (log_counter_ >= LOG_EVERY_N) {
      log_counter_ = 0;
      if (is_warning) {
        RCLCPP_WARN(get_logger(), "Battery: %.1f%%  (%.4f V)  [%s]",
                    percent, voltage, status.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Battery: %.1f%%  (%.4f V)  [%s]",
                    percent, voltage, status.c_str());
      }
    }

    last_percent_ = percent;
  }

  int   i2c_fd_{-1};
  int   log_counter_;
  float last_percent_;
  std::string last_status_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_percent_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BatteryMonitorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Node failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}