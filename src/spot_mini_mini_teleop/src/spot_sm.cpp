/// \file spot_sm.cpp
/// \brief Spot Mini Mini State Machine Node (ROS2)
///
/// Converted from ROS1 spot_sm.cpp
///
/// SUBSCRIBES:
///   /teleop      (geometry_msgs/Twist)   — joystick commands from teleop_node
///   /estop       (std_msgs/Bool)         — emergency stop
///
/// PUBLISHES:
///   /mini_cmd    (spot_mini_mini_interfaces/MiniCmd) — command to policy runner
///
/// SERVICES:
///   /switch_movement (std_srvs/Empty)    — toggle Stepping/Viewing mode

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "spot_mini_mini_interfaces/msg/mini_cmd.hpp"
#include "spot_mini_mini_teleop/spot.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class SpotStateMachine : public rclcpp::Node
{
public:
    SpotStateMachine()
    : Node("spot_sm_node"),
      spot_mini_(),          // initialize Spot command handler
      motion_flag_(false),
      ESTOP_(false)
    {
        // ── Parameters ────────────────────────────────────────────────
        // ROS1: nh_.getParam("frequency", frequency)
        // ROS2: declare then get
        this->declare_parameter("frequency", 5.0);
        this->declare_parameter("timeout",   1.0);

        double frequency = this->get_parameter("frequency").as_double();
        timeout_sec_     = this->get_parameter("timeout").as_double();

        RCLCPP_INFO(this->get_logger(), "STARTING NODE: spot_mini State Machine");
        RCLCPP_INFO(this->get_logger(), "Frequency: %.1f Hz | Timeout: %.1f s",
                    frequency, timeout_sec_);

        // ── Subscribers ───────────────────────────────────────────────
        // ROS1: nh.subscribe("teleop", 1, teleop_callback)
        // ROS2: create_subscription with lambda
        teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "teleop", 1,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                this->teleop_callback(msg);
            }
        );

        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "estop", 1,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                this->estop_callback(msg);
            }
        );

        // ── Publisher ─────────────────────────────────────────────────
        // ROS1: nh.advertise<mini_ros::MiniCmd>("mini_cmd", 1)
        // ROS2: create_publisher
        mini_pub_ = this->create_publisher<spot_mini_mini_interfaces::msg::MiniCmd>(
            "mini_cmd", 1
        );

        // ── Service Server ────────────────────────────────────────────
        // ROS1: nh.advertiseService("switch_movement", swm_callback)
        // ROS2: create_service with lambda
        switch_movement_srv_ = this->create_service<std_srvs::srv::Empty>(
            "switch_movement",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>)
            {
                this->switch_movement_callback();
            }
        );

        // ── Timer (replaces ROS1 while loop + rate.sleep()) ───────────
        // ROS1: ros::Rate rate(frequency); while(ros::ok()) { ... rate.sleep(); }
        // ROS2: timer callback at same frequency
        auto period = std::chrono::duration<double>(1.0 / frequency);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { this->timer_callback(); }
        );

        // Initialize timestamps
        last_time_    = this->get_clock()->now();
        current_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "State machine ready.");
    }

private:

    // ── Teleop Callback ───────────────────────────────────────────────
    // ROS1: void teleop_callback(const geometry_msgs::Twist &tw)
    // ROS2: SharedPtr instead of const ref
    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr tw)
    {
        spot_mini_.update_command(
            tw->linear.x,   // vx  — step length or pitch
            tw->linear.y,   // vy  — lateral fraction or roll
            tw->linear.z,   // z   — height
            tw->angular.z,  // w   — yaw rate or yaw
            tw->angular.x,  // wx  — right bumper (faster)
            tw->angular.y   // wy  — left bumper  (slower)
        );
    }

    // ── E-STOP Callback ───────────────────────────────────────────────
    void estop_callback(const std_msgs::msg::Bool::SharedPtr estop)
    {
        if (estop->data)
        {
            spot_mini_.update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            motion_flag_ = true;

            if (!ESTOP_)
            {
                RCLCPP_ERROR(this->get_logger(), "ENGAGING MANUAL E-STOP!");
                ESTOP_ = true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "DIS-ENGAGING MANUAL E-STOP!");
                ESTOP_ = false;
            }
        }

        last_time_ = this->get_clock()->now();
    }

    // ── Switch Movement Service Callback ──────────────────────────────
    // ROS1: bool swm_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    // ROS2: void, uses shared_ptr args (we don't need to use them)
    void switch_movement_callback()
    {
        spot_mini_.switch_movement();
        motion_flag_ = true;

        // Log the new mode
        spot::SpotCommand cmd = spot_mini_.return_command();
        if (cmd.movement == spot::Stepping)
        {
            RCLCPP_INFO(this->get_logger(),
                "SWITCHED TO STEPPING MODE (vx | vy | w | z)");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "SWITCHED TO VIEWING MODE (roll | pitch | yaw | z)");
        }
    }

    // ── Main Timer Callback (replaces while loop) ─────────────────────
    // ROS1: while(ros::ok()) { ros::spinOnce(); ... publish; rate.sleep(); }
    // ROS2: timer fires at same frequency, rclcpp handles spinning
    void timer_callback()
    {
        current_time_ = this->get_clock()->now();
        double elapsed = (current_time_ - last_time_).seconds();

        spot::SpotCommand cmd = spot_mini_.return_command();

        // Build MiniCmd message
        spot_mini_mini_interfaces::msg::MiniCmd mini_cmd;

        // Condition: send real command only if not flagged, not timed out, not E-STOP
        if (!motion_flag_ && !(elapsed > timeout_sec_) && !ESTOP_)
        {
            mini_cmd.x_velocity = cmd.x_velocity;
            mini_cmd.y_velocity = cmd.y_velocity;
            mini_cmd.rate       = cmd.rate;
            mini_cmd.roll       = cmd.roll;
            mini_cmd.pitch      = cmd.pitch;
            mini_cmd.yaw        = cmd.yaw;
            mini_cmd.z          = cmd.z;
            mini_cmd.faster     = cmd.faster;
            mini_cmd.slower     = cmd.slower;

            // Convert enum to string
            // ROS1: if (cmd.motion == spot::Go) mini_cmd.motion = "Go"
            // ROS2: identical logic
            mini_cmd.motion   = (cmd.motion   == spot::Go)       ? "Go"       : "Stop";
            mini_cmd.movement = (cmd.movement == spot::Stepping)  ? "Stepping" : "Viewing";
        }
        else
        {
            // Stopped state — zero everything
            mini_cmd.x_velocity = 0.0;
            mini_cmd.y_velocity = 0.0;
            mini_cmd.rate       = 0.0;
            mini_cmd.roll       = 0.0;
            mini_cmd.pitch      = 0.0;
            mini_cmd.yaw        = 0.0;
            mini_cmd.z          = 0.0;
            mini_cmd.faster     = 0.0;
            mini_cmd.slower     = 0.0;
            mini_cmd.motion     = "Stop";
            mini_cmd.movement   = "Stepping";
        }

        // Timeout warning
        if (elapsed > timeout_sec_)
        {
            RCLCPP_ERROR(this->get_logger(), "TIMEOUT...ENGAGING E-STOP!");
        }

        mini_pub_->publish(mini_cmd);
        motion_flag_ = false;
    }

    // ── Member Variables ──────────────────────────────────────────────
    spot::Spot spot_mini_;

    bool motion_flag_;
    bool ESTOP_;
    double timeout_sec_;

    rclcpp::Time current_time_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       estop_sub_;
    rclcpp::Publisher<spot_mini_mini_interfaces::msg::MiniCmd>::SharedPtr mini_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr           switch_movement_srv_;
    rclcpp::TimerBase::SharedPtr                               timer_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // ROS1: manual while loop
    // ROS2: spin handles everything via timer callbacks
    rclcpp::spin(std::make_shared<SpotStateMachine>());

    rclcpp::shutdown();
    return 0;
}