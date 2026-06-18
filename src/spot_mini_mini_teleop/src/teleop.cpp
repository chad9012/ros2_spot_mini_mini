#include "spot_mini_mini_teleop/teleop.hpp"

namespace tele
{
    Teleop::Teleop(const int & linear_x, const int & linear_y, const int & linear_z,
                   const int & angular, const double & l_scale, const double & a_scale,
                   const int & LB, const int & RB, const int & B_scale, const int & LT,
                   const int & RT, const int & UD, const int & LR,
                   const int & sw, const int & es)
    {
        linear_x_ = linear_x;
        linear_y_ = linear_y;
        linear_z_ = linear_z;
        angular_ = angular;
        l_scale_ = l_scale;
        a_scale_ = a_scale;
        RB_ = RB;
        LB_ = LB;
        B_scale_ = B_scale;
        RT_ = RT;
        LT_ = LT;
        sw_ = sw;
        es_ = es;
        UD_ = UD;
        LR_ = LR;

        switch_trigger = false;
        ESTOP = false;
        updown = 0;
        leftright = 0;
        left_bump = false;
        right_bump = false;
    }

    void Teleop::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
    {
        // ROS 2 signatures protect bounds check safely
        if(joy->axes.size() > (size_t)std::max({linear_x_, linear_y_, linear_z_, angular_, RB_, LB_, UD_, LR_})) {
            twist.linear.x = l_scale_ * joy->axes[linear_x_];
            twist.linear.y = l_scale_ * joy->axes[linear_y_];
            twist.linear.z = -l_scale_ * joy->axes[linear_z_];
            twist.angular.z = a_scale_ * joy->axes[angular_];
            twist.angular.x = B_scale_ * joy->axes[RB_];
            twist.angular.y = B_scale_ * joy->axes[LB_];
            updown = static_cast<int>(joy->axes[UD_]);
            leftright = static_cast<int>(-joy->axes[LR_]);
        }

        if(joy->buttons.size() > (size_t)std::max({sw_, es_, LT_, RT_})) {
            switch_trigger = joy->buttons[sw_];
            ESTOP = joy->buttons[es_];
            left_bump = joy->buttons[LT_];
            right_bump = joy->buttons[RT_];
        }
    }

    geometry_msgs::msg::Twist Teleop::return_twist()
    {
        return twist;
    }

    bool Teleop::return_trigger()
    {
        return switch_trigger;
    }

    bool Teleop::return_estop()
    {
        return ESTOP;
    }

    spot_mini_mini_interfaces::msg::JoyButtons Teleop::return_buttons()
    {
        spot_mini_mini_interfaces::msg::JoyButtons jb;
        jb.updown = updown;
        jb.leftright = leftright;
        jb.left_bump = left_bump;
        jb.right_bump = right_bump;
        return jb;
    }   
}