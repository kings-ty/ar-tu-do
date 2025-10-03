#include "keyboard_controller.h"
#include <algorithm>
#include <cmath>
#include <chrono>

double map(double value, double in_lower, double in_upper, double out_lower, double out_upper)
{
    return out_lower + (out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower);
}

/**
 * Class constructor that sets up a publisher for the drive parameters topic, creates a window and starts a timer for
 * the main loop
 * */
KeyboardController::KeyboardController() : Node("keyboard_controller")
{
    assert(m_key_codes.size() == KEY_COUNT && "KEY_CODES needs to have KEY_COUNT many elements.");
    assert(this->m_key_pressed_state.size() == KEY_COUNT &&
           "m_key_pressed_state needs to have KEY_COUNT many elements.");

    this->m_drive_parameters_publisher =
        this->create_publisher<drive_msgs::msg::DriveParam>(TOPIC_DRIVE_PARAMETERS, 1);
    this->m_enable_manual_publisher = this->create_publisher<std_msgs::msg::Header>(TOPIC_HEARTBEAT_MANUAL, 1);
    this->m_enable_autonomous_publisher = this->create_publisher<std_msgs::msg::Header>(TOPIC_HEARTBEAT_AUTONOMOUS, 1);
    this->m_drive_mode_subscriber =
        this->create_subscription<std_msgs::msg::Int32>(TOPIC_DRIVE_MODE, 1, 
            std::bind(&KeyboardController::driveModeCallback, this, std::placeholders::_1));

    this->loadImages();
    this->createWindow();

    auto tick_duration = std::chrono::milliseconds(static_cast<int>(1000.0 / PARAMETER_UPDATE_FREQUENCY));
    this->m_timer = this->create_wall_timer(tick_duration, std::bind(&KeyboardController::timerCallback, this));

    this->declareParameters();
}

KeyboardController::~KeyboardController()
{
    SDL_DestroyWindow(this->m_window);
    SDL_FreeSurface(this->m_image_locked);
    SDL_FreeSurface(this->m_image_manual);
    SDL_FreeSurface(this->m_image_autonomous);
    SDL_Quit();
}

void KeyboardController::createWindow()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("teleoperation");
    std::string icon_filename = package_path + std::string("/img/wasd.bmp");

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        throw std::runtime_error("Could not initialize SDL: " + std::string(SDL_GetError()));
    }
    this->m_window = SDL_CreateWindow("Keyboard teleoperation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 580,
                                      128, SDL_WINDOW_RESIZABLE);

    auto window_surface = SDL_GetWindowSurface(this->m_window);
    SDL_DisplayMode display_mode;
    SDL_GetCurrentDisplayMode(0, &display_mode);
    // clang-format off
    SDL_SetWindowPosition(this->m_window,
        display_mode.w - window_surface->w - SCREEN_EDGE_MARGIN,
        display_mode.h - window_surface->h - SCREEN_EDGE_MARGIN);
    // clang-format on

    SDL_Surface* icon = SDL_LoadBMP(icon_filename.c_str());
    if (icon != NULL)
    {
        SDL_SetWindowIcon(this->m_window, icon);
        SDL_FreeSurface(icon);
    }
}

void KeyboardController::pollWindowEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYUP || event.type == SDL_KEYDOWN)
        {
            for (size_t i = 0; i < KEY_COUNT; i++)
            {
                if ((int)event.key.keysym.sym == (int)m_key_codes[i])
                {
                    this->m_key_pressed_state[i] = event.type == SDL_KEYDOWN;
                }
            }
        }
        else if (event.type == SDL_QUIT)
        {
            rclcpp::shutdown();
        }
        else if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_EXPOSED)
        {
            this->updateWindow();
        }
    }
}

void KeyboardController::updateWindow()
{
    SDL_Surface* surface = SDL_GetWindowSurface(this->m_window);
    SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0, 0, 0));
    switch (this->m_drive_mode)
    {
        case DriveMode::LOCKED:
            SDL_BlitSurface(this->m_image_locked, NULL, surface, NULL);
            break;
        case DriveMode::MANUAL:
            SDL_BlitSurface(this->m_image_manual, NULL, surface, NULL);
            break;
        case DriveMode::AUTONOMOUS:
            SDL_BlitSurface(this->m_image_autonomous, NULL, surface, NULL);
            break;
    }
    SDL_UpdateWindowSurface(this->m_window);
}

void KeyboardController::loadImages()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("teleoperation");
    std::string locked_filename = package_path + std::string("/img/locked.bmp");
    this->m_image_locked = SDL_LoadBMP(locked_filename.c_str());
    std::string manual_filename = package_path + std::string("/img/manual.bmp");
    this->m_image_manual = SDL_LoadBMP(manual_filename.c_str());
    std::string autonomous_filename = package_path + std::string("/img/autonomous.bmp");
    this->m_image_autonomous = SDL_LoadBMP(autonomous_filename.c_str());
}

/**
 * This method is called at each tick of the timer. It updates the keyboard state and the drive parameters and publishes
 * them to the ROS topic.
 * */
void KeyboardController::timerCallback()
{
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    double delta_time = std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;

    this->pollWindowEvents();
    this->updateDriveParameters(delta_time);
    this->publishDriveParameters();
    this->updateDeadMansSwitch();
}

std_msgs::msg::Header createHeartbeatMessage()
{
    std_msgs::msg::Header message;
    message.stamp = rclcpp::Clock().now();
    return message;
}

/**
 *  Checks if the Dead Man's Switch key is pressed and publish the Dead Man's Switch message
 */
void KeyboardController::updateDeadMansSwitch()
{
    if (this->m_key_pressed_state[(size_t)KeyIndex::ENABLE_MANUAL])
    {
        this->m_enable_manual_publisher->publish(createHeartbeatMessage());
    }
    if (this->m_key_pressed_state[(size_t)KeyIndex::ENABLE_AUTONOMOUS])
    {
        this->m_enable_autonomous_publisher->publish(createHeartbeatMessage());
    }
}

/**
 *  This updates the drive parameters based on which keys are currently pressed and how much time passed since the last
 * update.
 *  If no keys are pressed, the steering angle and target velocity will move back to their default values over time.
 */
void KeyboardController::updateDriveParameters(double delta_time)
{
// Disable warnings about equality comparisons for floats.
// Equality comparisons are ok here because the variables are assigned the exact values that we compare them against.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    double steer = this->m_key_pressed_state[(size_t)KeyIndex::STEER_LEFT]
        ? -1
        : (this->m_key_pressed_state[(size_t)KeyIndex::STEER_RIGHT] ? +1 : 0);
    double throttle = this->m_key_pressed_state[(size_t)KeyIndex::ACCELERATE]
        ? +1
        : (this->m_key_pressed_state[(size_t)KeyIndex::DECELERATE] ? -1 : 0);

    double steer_limit = map(std::abs(this->m_velocity), 0, m_max_throttle, 1, m_fast_steer_limit);
    double angle_update = steer * delta_time * m_steering_speed;
    this->m_angle = std::clamp(this->m_angle + angle_update, -steer_limit, +steer_limit);
    double velocity_update = throttle * delta_time * (this->m_velocity * throttle > 0 ? m_acceleration : m_braking);
    this->m_velocity = std::clamp(this->m_velocity + velocity_update, -m_max_throttle, +m_max_throttle);

    if (steer == 0 && this->m_angle != 0)
    {
        double sign = std::copysign(1.0, this->m_angle);
        this->m_angle -= m_steering_gravity * delta_time * sign;
        if (std::abs(this->m_angle) < m_steering_gravity * delta_time)
        {
            this->m_angle = 0;
        }
    }

    if (throttle == 0 && this->m_velocity != 0)
    {
        double sign = std::copysign(1.0, this->m_velocity);
        this->m_velocity -= m_throttle_gravity * delta_time * sign;
        if (std::abs(this->m_velocity) < m_throttle_gravity * delta_time)
        {
            this->m_velocity = 0;
        }
    }
#pragma GCC diagnostic pop
}

void KeyboardController::publishDriveParameters()
{
    auto drive_parameters = drive_msgs::msg::DriveParam();
    drive_parameters.velocity = this->m_velocity;
    drive_parameters.angle = this->m_angle;
    this->m_drive_parameters_publisher->publish(drive_parameters);
}

void KeyboardController::driveModeCallback(const std_msgs::msg::Int32::SharedPtr drive_mode_message)
{
    auto mode = (DriveMode)drive_mode_message->data;
    assert((mode == DriveMode::LOCKED || mode == DriveMode::MANUAL || mode == DriveMode::AUTONOMOUS) &&
           "Unknown drive mode.");
    if (this->m_drive_mode != mode)
    {
        this->m_drive_mode = mode;
        this->updateWindow();
    }
}

void KeyboardController::declareParameters()
{
    this->declare_parameter("steering_speed", m_steering_speed);
    this->declare_parameter("acceleration", m_acceleration);
    this->declare_parameter("braking", m_braking);
    this->declare_parameter("fast_steer_limit", m_fast_steer_limit);
    this->declare_parameter("steering_gravity", m_steering_gravity);
    this->declare_parameter("throttle_gravity", m_throttle_gravity);
    this->declare_parameter("max_throttle", m_max_throttle);

    // Get parameters
    this->get_parameter("steering_speed", m_steering_speed);
    this->get_parameter("acceleration", m_acceleration);
    this->get_parameter("braking", m_braking);
    this->get_parameter("fast_steer_limit", m_fast_steer_limit);
    this->get_parameter("steering_gravity", m_steering_gravity);
    this->get_parameter("throttle_gravity", m_throttle_gravity);
    this->get_parameter("max_throttle", m_max_throttle);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto keyboard_controller = std::make_shared<KeyboardController>();
    rclcpp::spin(keyboard_controller);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
