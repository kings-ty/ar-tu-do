#pragma once

#include "drive_mode.h"
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>

#include <drive_msgs/msg/drive_param.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";
constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";
constexpr const char* TOPIC_EMERGENCY_STOP = "/commands/emergency_stop";

class CarController : public rclcpp::Node
{
    public:
    CarController();

    private:
    rclcpp::Subscription<drive_msgs::msg::DriveParam>::SharedPtr m_drive_parameters_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_drive_mode_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_emergency_stop_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_speed_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_angle_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_brake_publisher;

    bool m_drive_param_lock;
    bool m_emergency_stop_lock;
    DriveMode m_current_drive_mode;

    /**
     * @brief deals with incomming drive param messages
     */
    void driveParametersCallback(const drive_msgs::msg::DriveParam::SharedPtr parameters);

    /**
     * @brief sets the current drive mode
     */
    void driveModeCallback(const std_msgs::msg::Int32::SharedPtr drive_mode_message);

    /**
     * @brief callback for the topic that enables / disables the motor
     */
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr drive_mode_message);

    /**
     * @brief takes a speed and angle, converts and forwards them to gazebo/focbox
     */
    void publishDriveParameters(double raw_speed, double raw_angle);

    /**
     * @brief takes speed and publishes it to gazebo/focbox
     */
    void publishSpeed(double speed);

    /**
     * @brief takes angle and publishes it to gazebo/focbox
     */
    void publishAngle(double angle);

    /**
     * @brief publishes a brake message that stops the car
     */
    void stop();
};
