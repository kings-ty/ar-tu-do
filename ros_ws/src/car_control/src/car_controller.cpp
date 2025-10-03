#include "car_controller.h"
#include "car_config.h"

#include <algorithm>

CarController::CarController()
    : Node("car_controller")
    , m_drive_param_lock{ true }
    , m_emergency_stop_lock{ true }
{
    this->m_drive_parameters_subscriber =
        this->create_subscription<drive_msgs::msg::DriveParam>(TOPIC_DRIVE_PARAM, 1,
            std::bind(&CarController::driveParametersCallback, this, std::placeholders::_1));
    this->m_drive_mode_subscriber =
        this->create_subscription<std_msgs::msg::Int32>(TOPIC_DRIVE_MODE, 1,
            std::bind(&CarController::driveModeCallback, this, std::placeholders::_1));
    this->m_emergency_stop_subscriber =
        this->create_subscription<std_msgs::msg::Bool>(TOPIC_EMERGENCY_STOP, 1,
            std::bind(&CarController::emergencyStopCallback, this, std::placeholders::_1));

    this->m_speed_publisher = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_angle_publisher = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FOCBOX_ANGLE, 1);
    this->m_brake_publisher = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FOCBOX_BRAKE, 1);
}

void CarController::driveParametersCallback(const drive_msgs::msg::DriveParam::SharedPtr parameters)
{
    this->publishDriveParameters((m_drive_param_lock || m_emergency_stop_lock) ? 0 : parameters->velocity,
                                 m_drive_param_lock ? 0 : parameters->angle);
}

void CarController::publishDriveParameters(double relative_speed, double relative_angle)
{
    double speed = relative_speed * car_config::MAX_RPM_ELECTRICAL;
    double angle = (relative_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2;

    this->publishSpeed(speed);
    this->publishAngle(angle);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "running: " << " | speed: " << speed << " | angle: " << angle);
}

void CarController::publishSpeed(double speed)
{
    auto speed_message = std_msgs::msg::Float64();
    speed_message.data = speed;
    this->m_speed_publisher->publish(speed_message);
}

void CarController::publishAngle(double angle)
{
    auto angle_message = std_msgs::msg::Float64();
    angle_message.data = angle;
    this->m_angle_publisher->publish(angle_message);
}

void CarController::driveModeCallback(const std_msgs::msg::Int32::SharedPtr drive_mode_message)
{
    this->m_current_drive_mode = (DriveMode)drive_mode_message->data;
    this->m_drive_param_lock = this->m_current_drive_mode == DriveMode::LOCKED;
    if (this->m_drive_param_lock)
        this->stop();
}

void CarController::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr emergency_stop_message)
{
    bool enable_emergency_stop = emergency_stop_message->data && this->m_current_drive_mode != DriveMode::MANUAL;
    this->m_emergency_stop_lock = enable_emergency_stop;
    if (this->m_emergency_stop_lock)
        this->stop();
}

void CarController::stop()
{
    this->publishSpeed(0);

    auto brake_message = std_msgs::msg::Float64();
    brake_message.data = 0;
    this->m_brake_publisher->publish(brake_message);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto car_controller = std::make_shared<CarController>();
    rclcpp::spin(car_controller);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
