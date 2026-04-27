#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class PointCloudCropFilter : public rclcpp::Node
{
public:
  PointCloudCropFilter() : Node("pointcloud_crop_filter")
  {
    // Declare parameters
    this->declare_parameter("min_x", -0.8);
    this->declare_parameter("max_x", 1.0);
    this->declare_parameter("min_y", -0.5);
    this->declare_parameter("max_y", 0.5);
    this->declare_parameter("min_z", -0.5);
    this->declare_parameter("max_z", 0.8);
    this->declare_parameter("input_topic", "/points");
    this->declare_parameter("output_topic", "/points_filtered");
    this->declare_parameter("marker_topic", "/crop_box_marker");
    this->declare_parameter("negative", true);  // true = remove inside box
    
    // Get parameters
    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    negative_ = this->get_parameter("negative").as_bool();
    
    // Create subscriber
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      std::bind(&PointCloudCropFilter::cloudCallback, this, std::placeholders::_1));
    
    // Create publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
    
    // Timer for marker publishing (1 Hz)
    marker_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PointCloudCropFilter::publishMarker, this));
    
    RCLCPP_INFO(this->get_logger(), "Point Cloud Crop Filter Started");
    RCLCPP_INFO(this->get_logger(), "Input: %s -> Output: %s", 
                input_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Crop Box: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    RCLCPP_INFO(this->get_logger(), "Mode: %s", 
                negative_ ? "Remove inside box" : "Keep inside box");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Create crop box filter
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(cloud);
    
    // Set crop box limits
    Eigen::Vector4f min_point(min_x_, min_y_, min_z_, 1.0);
    Eigen::Vector4f max_point(max_x_, max_y_, max_z_, 1.0);
    crop_filter.setMin(min_point);
    crop_filter.setMax(max_point);
    
    // Set negative flag (true = remove inside, false = keep inside)
    crop_filter.setNegative(negative_);
    
    // Apply filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_filter.filter(*filtered_cloud);
    
    // Convert back to ROS message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = msg->header;
    
    // Publish filtered cloud
    cloud_pub_->publish(output_msg);
  }
  
  void publishMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "crop_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Scale (line width)
    marker.scale.x = 0.02;
    
    // Red color with transparency
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    // Define 8 corners of the box
    geometry_msgs::msg::Point corners[8];
    corners[0].x = min_x_; corners[0].y = min_y_; corners[0].z = min_z_;  // 0: bottom-back-left
    corners[1].x = max_x_; corners[1].y = min_y_; corners[1].z = min_z_;  // 1: bottom-front-left
    corners[2].x = max_x_; corners[2].y = max_y_; corners[2].z = min_z_;  // 2: bottom-front-right
    corners[3].x = min_x_; corners[3].y = max_y_; corners[3].z = min_z_;  // 3: bottom-back-right
    corners[4].x = min_x_; corners[4].y = min_y_; corners[4].z = max_z_;  // 4: top-back-left
    corners[5].x = max_x_; corners[5].y = min_y_; corners[5].z = max_z_;  // 5: top-front-left
    corners[6].x = max_x_; corners[6].y = max_y_; corners[6].z = max_z_;  // 6: top-front-right
    corners[7].x = min_x_; corners[7].y = max_y_; corners[7].z = max_z_;  // 7: top-back-right
    
    // Bottom face edges
    marker.points.push_back(corners[0]); marker.points.push_back(corners[1]);
    marker.points.push_back(corners[1]); marker.points.push_back(corners[2]);
    marker.points.push_back(corners[2]); marker.points.push_back(corners[3]);
    marker.points.push_back(corners[3]); marker.points.push_back(corners[0]);
    
    // Top face edges
    marker.points.push_back(corners[4]); marker.points.push_back(corners[5]);
    marker.points.push_back(corners[5]); marker.points.push_back(corners[6]);
    marker.points.push_back(corners[6]); marker.points.push_back(corners[7]);
    marker.points.push_back(corners[7]); marker.points.push_back(corners[4]);
    
    // Vertical edges
    marker.points.push_back(corners[0]); marker.points.push_back(corners[4]);
    marker.points.push_back(corners[1]); marker.points.push_back(corners[5]);
    marker.points.push_back(corners[2]); marker.points.push_back(corners[6]);
    marker.points.push_back(corners[3]); marker.points.push_back(corners[7]);
    
    marker_pub_->publish(marker);
  }
  
  // Parameters
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  std::string input_topic_, output_topic_, marker_topic_;
  bool negative_;
  
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudCropFilter>());
  rclcpp::shutdown();
  return 0;
}
