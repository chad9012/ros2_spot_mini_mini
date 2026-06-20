#ifndef SPOT_MINI_MINI_TELEOP__TELEOP_HPP_
#define SPOT_MINI_MINI_TELEOP__TELEOP_HPP_

#include <vector>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "spot_mini_mini_interfaces/msg/joy_buttons.hpp"

namespace tele
{
    class Teleop
    {
    public:
        Teleop(const int & linear_x, const int & linear_y, const int & linear_z,
               const int & angular, const double & l_scale, const double & a_scale,
               const int & LB, const int & RB, const double & B_scale, const int & LT,
               const int & RT, const int & UD, const int & LR,
               const int & sw, const int & es);

        // Updated to use ROS 2 shared pointer message signatures
        void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

        geometry_msgs::msg::Twist return_twist();
        bool return_trigger();
        bool return_estop();
        spot_mini_mini_interfaces::msg::JoyButtons return_buttons();

    private:
        int linear_x_ = 0;
        int linear_y_ = 0;
        int linear_z_ = 0;
        int angular_= 0;
        int RB_ = 0;
        int LB_ = 0;
        int sw_ = 0;
        int es_ = 0;
        int RT_ = 0;
        int LT_ = 0;
        int UD_ = 0;
        int LR_ = 0;
        
        double l_scale_, a_scale_, B_scale_;
        
        geometry_msgs::msg::Twist twist;
        bool switch_trigger = false;
        bool ESTOP = false;
        int updown = 0;
        int leftright = 0;
        bool left_bump = false;
        bool right_bump = false;
    };
}

#endif // SPOT_MINI_MINI_TELEOP__TELEOP_HPP_