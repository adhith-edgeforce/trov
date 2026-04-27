#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class GpsOdomNode : public rclcpp::Node
{
public:
  GpsOdomNode() : Node("gps_odom_node")
  {
    declare_parameter("gps_topic", "/mavros/global_position/global");
    declare_parameter("imu_topic", "/imu/data");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("world_frame", "odom");
    declare_parameter("base_link_frame", "base_link");
    declare_parameter("zero_altitude", true);

    gps_topic_       = get_parameter("gps_topic").as_string();
    imu_topic_       = get_parameter("imu_topic").as_string();
    odom_topic_      = get_parameter("odom_topic").as_string();
    world_frame_     = get_parameter("world_frame").as_string();
    base_link_frame_ = get_parameter("base_link_frame").as_string();
    zero_altitude_   = get_parameter("zero_altitude").as_bool();

    auto gps_qos = rclcpp::QoS(10).best_effort();
    auto imu_qos = rclcpp::QoS(50).best_effort();

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_, gps_qos,
      std::bind(&GpsOdomNode::gps_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&GpsOdomNode::imu_callback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(50).reliable());

    // TF broadcaster intentionally removed — EKF publishes odom->base_link TF

    RCLCPP_INFO(get_logger(),
      "GpsOdomNode: gps=%s imu=%s -> odom=%s (no TF, EKF handles that)",
      gps_topic_.c_str(),
      imu_topic_.c_str(),
      odom_topic_.c_str());
  }

private:
  std::string gps_topic_, imu_topic_, odom_topic_;
  std::string world_frame_, base_link_frame_;
  bool zero_altitude_{true};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool have_origin_{false};
  double origin_lat_{0.0}, origin_lon_{0.0}, origin_alt_{0.0};

  geometry_msgs::msg::Quaternion last_orientation_;
  bool have_orientation_{false};

  static double deg2rad(double d)
  {
    return d * M_PI / 180.0;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (msg->orientation_covariance[0] < 0) {
      return;
    }

    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    q.normalize();

    last_orientation_.x = q.x();
    last_orientation_.y = q.y();
    last_orientation_.z = q.z();
    last_orientation_.w = q.w();

    have_orientation_ = true;
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status ==
        sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No GPS fix yet.");
      return;
    }

    if (!have_origin_) {
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      origin_alt_ = msg->altitude;
      have_origin_ = true;

      RCLCPP_INFO(get_logger(),
        "Origin set: lat=%.8f lon=%.8f alt=%.2f",
        origin_lat_, origin_lon_, origin_alt_);
    }

    double lat_rad  = deg2rad(msg->latitude);
    double lon_rad  = deg2rad(msg->longitude);
    double lat0_rad = deg2rad(origin_lat_);
    double lon0_rad = deg2rad(origin_lon_);

    double dlat = lat_rad - lat0_rad;
    double dlon = lon_rad - lon0_rad;

    const double R = 6378137.0;

    double x = dlon * cos(lat0_rad) * R;
    double y = dlat * R;
    double z = zero_altitude_ ? 0.0 : (msg->altitude - origin_alt_);

    nav_msgs::msg::Odometry odom;

    odom.header.stamp    = msg->header.stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id  = base_link_frame_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;

    if (have_orientation_) {
      odom.pose.pose.orientation = last_orientation_;
    } else {
      odom.pose.pose.orientation.w = 1.0;
    }

    // Pose covariance — use GPS reported covariance if valid
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;

    double var_x = msg->position_covariance[0];
    double var_y = msg->position_covariance[4];

    odom.pose.covariance[0]  = var_x > 0 ? var_x : 4.0;
    odom.pose.covariance[7]  = var_y > 0 ? var_y : 4.0;
    odom.pose.covariance[14] = 1e6;   // z not trusted
    odom.pose.covariance[21] = 1e6;   // roll not trusted
    odom.pose.covariance[28] = 1e6;   // pitch not trusted
    odom.pose.covariance[35] = 0.5;   // yaw (from IMU)

    // Twist — not measured, signal EKF to ignore it
    for (int i = 0; i < 36; i++) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0]  = -1;
    odom.twist.covariance[7]  = -1;
    odom.twist.covariance[14] = -1;
    odom.twist.covariance[21] = -1;
    odom.twist.covariance[28] = -1;
    odom.twist.covariance[35] = -1;

    odom_pub_->publish(odom);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <cmath>

// class GpsOdomNode : public rclcpp::Node
// {
// public:
//   GpsOdomNode() : Node("gps_odom_node")
//   {
//     declare_parameter("gps_topic", "/mavros/global_position/global");
//     declare_parameter("imu_topic", "/imu/data");
//     declare_parameter("odom_topic", "/odom");
//     declare_parameter("world_frame", "odom");
//     declare_parameter("base_link_frame", "base_link");
//     declare_parameter("publish_tf", false);
//     declare_parameter("zero_altitude", true);

//     gps_topic_       = get_parameter("gps_topic").as_string();
//     imu_topic_       = get_parameter("imu_topic").as_string();
//     odom_topic_      = get_parameter("odom_topic").as_string();
//     world_frame_     = get_parameter("world_frame").as_string();
//     base_link_frame_ = get_parameter("base_link_frame").as_string();
//     publish_tf_      = get_parameter("publish_tf").as_bool();
//     zero_altitude_   = get_parameter("zero_altitude").as_bool();

//     auto gps_qos = rclcpp::QoS(10).best_effort();
//     auto imu_qos = rclcpp::QoS(50).best_effort();

//     gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
//       gps_topic_, gps_qos,
//       std::bind(&GpsOdomNode::gps_callback, this, std::placeholders::_1));

//     imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
//       imu_topic_, imu_qos,
//       std::bind(&GpsOdomNode::imu_callback, this, std::placeholders::_1));

//     odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
//       odom_topic_,
//       rclcpp::QoS(50).reliable());

//     if (publish_tf_) {
//       tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
//     }

//     RCLCPP_INFO(get_logger(),
//       "GpsOdomNode: gps=%s imu=%s -> odom=%s",
//       gps_topic_.c_str(),
//       imu_topic_.c_str(),
//       odom_topic_.c_str());
//   }

// private:
//   std::string gps_topic_, imu_topic_, odom_topic_;
//   std::string world_frame_, base_link_frame_;
//   bool publish_tf_{false}, zero_altitude_{true};

//   rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//   bool have_origin_{false};
//   double origin_lat_{0.0}, origin_lon_{0.0}, origin_alt_{0.0};

//   geometry_msgs::msg::Quaternion last_orientation_;
//   bool have_orientation_{false};

//   static double deg2rad(double d)
//   {
//     return d * M_PI / 180.0;
//   }

//   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
//   {
//     // Ignore invalid IMU data
//     if (msg->orientation_covariance[0] < 0) {
//       return;
//     }

//     tf2::Quaternion q(
//       msg->orientation.x,
//       msg->orientation.y,
//       msg->orientation.z,
//       msg->orientation.w);

//     q.normalize();

//     last_orientation_.x = q.x();
//     last_orientation_.y = q.y();
//     last_orientation_.z = q.z();
//     last_orientation_.w = q.w();

//     have_orientation_ = true;
//   }

//   void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
//   {
//     if (msg->status.status ==
//         sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//                            "No GPS fix yet.");
//       return;
//     }

//     if (!have_origin_) {
//       origin_lat_ = msg->latitude;
//       origin_lon_ = msg->longitude;
//       origin_alt_ = msg->altitude;
//       have_origin_ = true;

//       RCLCPP_INFO(get_logger(),
//         "Origin set: lat=%.8f lon=%.8f alt=%.2f",
//         origin_lat_, origin_lon_, origin_alt_);
//     }

//     double lat_rad  = deg2rad(msg->latitude);
//     double lon_rad  = deg2rad(msg->longitude);
//     double lat0_rad = deg2rad(origin_lat_);
//     double lon0_rad = deg2rad(origin_lon_);

//     double dlat = lat_rad - lat0_rad;
//     double dlon = lon_rad - lon0_rad;

//     const double R = 6378137.0;

//     double x = dlon * cos(lat0_rad) * R;
//     double y = dlat * R;
//     double z = zero_altitude_ ? 0.0 : (msg->altitude - origin_alt_);

//     nav_msgs::msg::Odometry odom;

//     odom.header.stamp = msg->header.stamp;
//     odom.header.frame_id = world_frame_;
//     odom.child_frame_id = base_link_frame_;

//     odom.pose.pose.position.x = x;
//     odom.pose.pose.position.y = y;
//     odom.pose.pose.position.z = z;

//     if (have_orientation_) {
//       odom.pose.pose.orientation = last_orientation_;
//     } else {
//       odom.pose.pose.orientation.w = 1.0;
//     }

//     // ----------- COVARIANCE FIXES -----------

//     for (int i = 0; i < 36; i++) {
//       odom.pose.covariance[i] = 0.0;
//     }

//     double var_x = msg->position_covariance[0];
//     double var_y = msg->position_covariance[4];

//     odom.pose.covariance[0]  = var_x > 0 ? var_x : 4.0;
//     odom.pose.covariance[7]  = var_y > 0 ? var_y : 4.0;
//     odom.pose.covariance[14] = 1e6;

//     // IMPORTANT: don't trust yaw too much
//     odom.pose.covariance[21] = 1e6;
//     odom.pose.covariance[28] = 1e6;
//     odom.pose.covariance[35] = 0.5;   // ← FIXED (was too small before)

//     // Ignore twist completely
//     for (int i = 0; i < 36; i++) {
//       odom.twist.covariance[i] = 0.0;
//     }

//     odom.twist.covariance[0]  = -1;
//     odom.twist.covariance[7]  = -1;
//     odom.twist.covariance[14] = -1;
//     odom.twist.covariance[21] = -1;
//     odom.twist.covariance[28] = -1;
//     odom.twist.covariance[35] = -1;

//     odom_pub_->publish(odom);

//     // ----------- TF -----------

//     if (publish_tf_ && tf_broadcaster_ && have_orientation_) {
//       geometry_msgs::msg::TransformStamped tf;

//       tf.header.stamp = msg->header.stamp;
//       tf.header.frame_id = world_frame_;
//       tf.child_frame_id = base_link_frame_;

//       tf.transform.translation.x = x;
//       tf.transform.translation.y = y;
//       tf.transform.translation.z = z;
//       tf.transform.rotation = odom.pose.pose.orientation;

//       tf_broadcaster_->sendTransform(tf);
//     }
//   }
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<GpsOdomNode>();

//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }