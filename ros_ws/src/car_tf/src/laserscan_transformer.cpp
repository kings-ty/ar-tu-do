#include "laserscan_transformer.h"
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/transform_listener.h>

LaserscanTransformer::LaserscanTransformer() : Node("laserscan_transformer")
{
    // Initialize TF2 buffer and listener
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Create subscription and publisher
    m_laserscan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        TOPIC_LASER_SCAN, 100,
        std::bind(&LaserscanTransformer::scanCallback, this, std::placeholders::_1));
    
    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        TOPIC_LASER_SCAN_POINTCLOUD, 100);
}

void LaserscanTransformer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan)
{
    try {
        sensor_msgs::msg::PointCloud2 pointcloud;
        m_projector.transformLaserScanToPointCloud(MODEL_BASE_LINK, *laserscan, pointcloud, *m_tf_buffer);
        m_pointcloud_publisher->publish(pointcloud);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform laser scan: %s", ex.what());
    }
}
