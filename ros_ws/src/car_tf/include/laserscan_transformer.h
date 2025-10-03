#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

constexpr const char* TOPIC_LASER_SCAN = "/racer/laser/scan";
constexpr const char* TOPIC_LASER_SCAN_POINTCLOUD = "/racer/laser/tf_pointcloud";
constexpr const char* MODEL_BASE_LINK = "base_link";

/**
 * @brief This converter class converts a 2D laser scan
 * as defined by sensor_msgs/LaserScan into a point
 * cloud as defined by sensor_msgs/PointCloud2.
 *
 * The main purpose of this class is the transformation
 * of the LaserScan to the "base_link" of the racer model.
 */
class LaserscanTransformer : public rclcpp::Node
{
    public:
    LaserscanTransformer();

    private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laserscan_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    laser_geometry::LaserProjection m_projector;
    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan);
};