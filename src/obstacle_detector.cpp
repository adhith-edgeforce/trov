// File: obstacle_detector.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() : Node("obstacle_detector")
    {
        // Declare parameters with SMALLER defaults for desk testing
        this->declare_parameter("box_min_x", 0.2);      // Start 20cm in front
        this->declare_parameter("box_min_y", -0.9);     // 30cm left
        this->declare_parameter("box_min_z", -0.4);     // 20cm below
        this->declare_parameter("box_max_x", 2.0);      // End 80cm in front (60cm deep box)
        this->declare_parameter("box_max_y", 0.9);      // 30cm right
        this->declare_parameter("box_max_z", 1.0);      // 50cm above
        this->declare_parameter("min_points_threshold", 20);  // Lower threshold for small box
        this->declare_parameter("box_alpha", 0.3);

        // Get parameters
        box_min_x_ = this->get_parameter("box_min_x").as_double();
        box_min_y_ = this->get_parameter("box_min_y").as_double();
        box_min_z_ = this->get_parameter("box_min_z").as_double();
        box_max_x_ = this->get_parameter("box_max_x").as_double();
        box_max_y_ = this->get_parameter("box_max_y").as_double();
        box_max_z_ = this->get_parameter("box_max_z").as_double();
        min_points_threshold_ = this->get_parameter("min_points_threshold").as_int();
        box_alpha_ = this->get_parameter("box_alpha").as_double();

        // Subscribers and publishers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points", 10,
            std::bind(&ObstacleDetector::pointcloudCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/detection_box", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Detector Node Started");
        RCLCPP_INFO(this->get_logger(), "Detection Box: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                    box_min_x_, box_min_y_, box_min_z_, box_max_x_, box_max_y_, box_max_z_);
        RCLCPP_INFO(this->get_logger(), "Box size: %.2fm x %.2fm x %.2fm",
                    box_max_x_ - box_min_x_, box_max_y_ - box_min_y_, box_max_z_ - box_min_z_);
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS2 PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Create CropBox filter
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(cloud);
        
        Eigen::Vector4f min_point(box_min_x_, box_min_y_, box_min_z_, 1.0);
        Eigen::Vector4f max_point(box_max_x_, box_max_y_, box_max_z_, 1.0);
        
        crop_box.setMin(min_point);
        crop_box.setMax(max_point);

        // Get points inside the box
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        crop_box.filter(*cropped_cloud);

        int num_points = cropped_cloud->points.size();
        bool obstacle_detected = num_points > min_points_threshold_;

        // Print safety status
        if (obstacle_detected) {
            RCLCPP_WARN(this->get_logger(), 
                        "⚠️  NOT SAFE - Obstacle detected! Points in box: %d", num_points);
        } else {
            RCLCPP_INFO(this->get_logger(), 
                        "✓ SAFE - Clear path. Points in box: %d", num_points);
        }

        // Publish visualization marker (using the same frame as the pointcloud)
        publishMarker(msg->header, obstacle_detected);
    }

    void publishMarker(const std_msgs::msg::Header& header, bool obstacle_detected)
    {
        visualization_msgs::msg::Marker marker;
        
        // CRITICAL: Use the same frame_id as the lidar pointcloud
        // This makes the box move with the lidar sensor
        marker.header.frame_id = header.frame_id;  // Use lidar frame
        marker.header.stamp = this->now();
        marker.ns = "detection_box";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position (center of the box) - relative to lidar frame
        marker.pose.position.x = (box_min_x_ + box_max_x_) / 2.0;
        marker.pose.position.y = (box_min_y_ + box_max_y_) / 2.0;
        marker.pose.position.z = (box_min_z_ + box_max_z_) / 2.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Scale (dimensions of the box)
        marker.scale.x = box_max_x_ - box_min_x_;
        marker.scale.y = box_max_y_ - box_min_y_;
        marker.scale.z = box_max_z_ - box_min_z_;

        // Color (green if safe, red if obstacle)
        if (obstacle_detected) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker.color.a = box_alpha_;

        // Keep marker alive for a short time
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    double box_min_x_, box_min_y_, box_min_z_;
    double box_max_x_, box_max_y_, box_max_z_;
    int min_points_threshold_;
    double box_alpha_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}