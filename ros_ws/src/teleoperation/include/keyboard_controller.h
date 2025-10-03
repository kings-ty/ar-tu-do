#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_keycode.h>
#include <algorithm>
#include <array>
#include <memory>
#include <signal.h>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/header.hpp>
#include <drive_msgs/msg/drive_param.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "input/drive_param/keyboard";
constexpr const char* TOPIC_HEARTBEAT_MANUAL = "/input/heartbeat_manual";
constexpr const char* TOPIC_HEARTBEAT_AUTONOMOUS = "/input/heartbeat_autonomous";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";

enum class Keycode : int
{
    SPACE = SDLK_SPACE,
    A = SDLK_a,
    B = SDLK_b,
    D = SDLK_d,
    S = SDLK_s,
    W = SDLK_w,
};

enum class KeyIndex : size_t
{
    ACCELERATE = 0,
    DECELERATE = 2,
    STEER_LEFT = 1,
    STEER_RIGHT = 3,
    ENABLE_MANUAL = 4,
    ENABLE_AUTONOMOUS = 5
};

enum class DriveMode : int
{
    LOCKED = 0,
    MANUAL = 1,
    AUTONOMOUS = 2
};

constexpr double PARAMETER_UPDATE_FREQUENCY = 90;
constexpr int SCREEN_EDGE_MARGIN = 20;

class KeyboardController : public rclcpp::Node
{
    public:
    KeyboardController();
    KeyboardController(KeyboardController&&) = default;
    KeyboardController(const KeyboardController&) = default;
    ~KeyboardController();

    private:
    // How fast the steering value changes, in units per second
    double m_steering_speed = 6;
    // How fast the velocity changes, in units per second
    double m_acceleration = 0.4;
    // How fast the velocity changes when decelerating, in units per second
    double m_braking = 2;

    // MAX_STEERING is multiplied by this when travelling at maximum velocity, by 1.0 when resting and by an
    // interpolated value otherwise
    double m_fast_steer_limit = 0.6;

    // When no steering key is pressed, the steering value will change towards 0 at this rate, in units per second
    double m_steering_gravity = 2;
    // When no throttle key is pressed, the velocity will change towards 0 at this rate, in units per second
    double m_throttle_gravity = 3;

    double m_max_throttle = 0.35;

    static constexpr size_t KEY_COUNT = 6;
    std::array<Keycode, KEY_COUNT> m_key_codes = { { Keycode::W, Keycode::A, Keycode::S, Keycode::D, Keycode::SPACE,
                                                     Keycode::B } };

    rclcpp::Publisher<drive_msgs::msg::DriveParam>::SharedPtr m_drive_parameters_publisher;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr m_enable_manual_publisher;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr m_enable_autonomous_publisher;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_drive_mode_subscriber;

    SDL_Window* m_window;

    SDL_Surface* m_image_locked;
    SDL_Surface* m_image_manual;
    SDL_Surface* m_image_autonomous;

    std::array<bool, KEY_COUNT> m_key_pressed_state = { { false, false, false, false } };

    double m_velocity = 0;
    double m_angle = 0;

    rclcpp::TimerBase::SharedPtr m_timer;

    DriveMode m_drive_mode = DriveMode::LOCKED;

    void pollWindowEvents();

    void updateDriveParameters(double delta_time);

    void publishDriveParameters();

    void updateDeadMansSwitch();

    void createWindow();
    void timerCallback();
    void driveModeCallback(const std_msgs::msg::Int32::SharedPtr drive_mode_message);
    void updateWindow();
    void loadImages();

    void declareParameters();
};
