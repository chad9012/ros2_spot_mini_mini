#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "spot_mini_mini_interfaces/msg/joy_buttons.hpp"
#include "spot_mini_mini_teleop/teleop.hpp"

#include <chrono>
#include <memory>
#include <algorithm>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_node");

    RCLCPP_INFO(node->get_logger(), "STARTING NODE: Teleoperation (ROS 2)");

    // Declare and retrieve parameters safely
    node->declare_parameter("frequency", 60.0);
    node->declare_parameter("axis_linear_x", 4);
    node->declare_parameter("axis_linear_y", 3);
    node->declare_parameter("axis_linear_z", 1);
    node->declare_parameter("axis_angular", 0);
    node->declare_parameter("scale_linear", 1.0);
    node->declare_parameter("scale_angular", 1.0);
    node->declare_parameter("scale_bumper", 1.0);
    node->declare_parameter("button_switch", 0);
    node->declare_parameter("button_estop", 1);
    node->declare_parameter("rb", 5);
    node->declare_parameter("lb", 2);
    node->declare_parameter("rt", 5);
    node->declare_parameter("lt", 4);
    node->declare_parameter("updown", 7);
    node->declare_parameter("leftright", 6);
    node->declare_parameter("debounce_thresh", 0.15);

    double frequency = node->get_parameter("frequency").as_double();
    int linear_x = node->get_parameter("axis_linear_x").as_int();
    int linear_y = node->get_parameter("axis_linear_y").as_int();
    int linear_z = node->get_parameter("axis_linear_z").as_int();
    int angular = node->get_parameter("axis_angular").as_int();
    double l_scale = node->get_parameter("scale_linear").as_double();
    double a_scale = node->get_parameter("scale_angular").as_double();
    double B_scale = node->get_parameter("scale_bumper").as_double();
    int sw = node->get_parameter("button_switch").as_int();
    int es = node->get_parameter("button_estop").as_int();
    int RB = node->get_parameter("rb").as_int();
    int LB = node->get_parameter("lb").as_int();
    int RT = node->get_parameter("rt").as_int();
    int LT = node->get_parameter("lt").as_int();
    int UD = node->get_parameter("updown").as_int();
    int LR = node->get_parameter("leftright").as_int();
    double debounce_thresh = node->get_parameter("debounce_thresh").as_double();

    // Initialize your library instance
    tele::Teleop teleop(linear_x, linear_y, linear_z, angular,
                        l_scale, a_scale, LB, RB, B_scale, LT,
                        RT, UD, LR, sw, es);

    // Initialize Service Client
    auto switch_movement_client = node->create_client<std_srvs::srv::Empty>("switch_movement");

    // Initialize Publishers
    auto estop_pub = node->create_publisher<std_msgs::msg::Bool>("estop", 1);
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("teleop", 1);
    auto jb_pub = node->create_publisher<spot_mini_mini_interfaces::msg::JoyButtons>("joybuttons", 1);

    // Initialize Subscriber using a clean C++ Lambda
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, [&teleop](const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
            teleop.joyCallback(msg);
        });

    rclcpp::Rate rate(frequency);
    auto last_time = node->get_clock()->now();

    while (rclcpp::ok())
    {
        // Process incoming joystick callbacks
        rclcpp::spin_some(node);
        auto current_time = node->get_clock()->now();

        std_msgs::msg::Bool estop;
        estop.data = teleop.return_estop();
        double elapsed_sec = (current_time - last_time).seconds();

        if (estop.data && elapsed_sec >= debounce_thresh) {
            RCLCPP_INFO(node->get_logger(), "SENDING E-STOP COMMAND!");
            last_time = node->get_clock()->now();
        } 
        else if (!teleop.return_trigger()) {
            vel_pub->publish(teleop.return_twist());
            estop.data = false;
        } 
        else if (elapsed_sec >= debounce_thresh) {
            if (switch_movement_client->service_is_ready()) {
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                switch_movement_client->async_send_request(request);
            }
            estop.data = false;
            last_time = node->get_clock()->now();
        }

        jb_pub->publish(teleop.return_buttons());
        estop_pub->publish(estop);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}