#include "spot_mini_mini_teleop/spot.hpp"

// ROS2 logger for warnings — passed in from node
// We use a simple approach: spot.cpp is pure logic, no ROS dependency
// Logging handled in spot_sm.cpp which has access to the node logger
#include <iostream>

namespace spot
{
    Spot::Spot()
    {
        cmd.x_velocity = 0.0;
        cmd.y_velocity = 0.0;
        cmd.rate       = 0.0;
        cmd.roll       = 0.0;
        cmd.pitch      = 0.0;
        cmd.yaw        = 0.0;
        cmd.z          = 0.0;
        cmd.faster     = 0.0;
        cmd.slower     = 0.0;
        cmd.motion     = Stop;
        cmd.movement   = Viewing;
    }

    void Spot::update_command(const double & vx, const double & vy, const double & z,
                              const double & w,  const double & wx, const double & wy)
    {
        // If all inputs are near zero — stop
        if (almost_equal(vx, 0.0) && almost_equal(vy, 0.0) &&
            almost_equal(z,  0.0) && almost_equal(w,  0.0))
        {
            cmd.motion     = Stop;
            cmd.x_velocity = 0.0;
            cmd.y_velocity = 0.0;
            cmd.rate       = 0.0;
            cmd.roll       = 0.0;
            cmd.pitch      = 0.0;
            cmd.yaw        = 0.0;
            cmd.z          = 0.0;
            cmd.faster     = 0.0;
            cmd.slower     = 0.0;
        }
        else
        {
            cmd.motion = Go;

            if (cmd.movement == Stepping)
            {
                // Stepping mode: joystick → velocity + yaw rate + height
                cmd.x_velocity = vx;
                cmd.y_velocity = vy;
                cmd.rate       = w;
                cmd.z          = z;
                cmd.roll       = 0.0;
                cmd.pitch      = 0.0;
                cmd.yaw        = 0.0;
                // Bumpers change clearance height
                // wx = right bumper, wy = left bumper
                cmd.faster     =  (1.0 - wx);
                cmd.slower     = -(1.0 - wy);
            }
            else
            {
                // Viewing mode: joystick → body pose (RPY + height)
                cmd.x_velocity = 0.0;
                cmd.y_velocity = 0.0;
                cmd.rate       = 0.0;
                cmd.roll       = vy;   // left/right → roll
                cmd.pitch      = vx;   // up/down    → pitch
                cmd.yaw        = w;    // rotate     → yaw
                cmd.z          = z;
                cmd.faster     = 0.0;
                cmd.slower     = 0.0;
            }
        }
    }

    void Spot::switch_movement()
    {
        // Safety check — must be stopped before switching mode
        if (!almost_equal(cmd.x_velocity, 0.0) ||
            !almost_equal(cmd.y_velocity, 0.0) ||
            !almost_equal(cmd.rate,       0.0))
        {
            // Not at zero — force stop, warn user
            // Note: ROS1 used ROS_WARN here
            // ROS2: logging done in spot_sm.cpp via node logger
            std::cerr << "[SPOT] WARNING: Robot not stopped! "
                      << "vx=" << cmd.x_velocity
                      << " vy=" << cmd.y_velocity
                      << " rate=" << cmd.rate
                      << " — forcing stop before switch." << std::endl;

            cmd.motion     = Stop;
            cmd.x_velocity = 0.0;
            cmd.y_velocity = 0.0;
            cmd.rate       = 0.0;
            cmd.roll       = 0.0;
            cmd.pitch      = 0.0;
            cmd.yaw        = 0.0;
            cmd.z          = 0.0;
            cmd.faster     = 0.0;
            cmd.slower     = 0.0;
        }
        else
        {
            // Safe to switch — reset all velocities first
            cmd.x_velocity = 0.0;
            cmd.y_velocity = 0.0;
            cmd.rate       = 0.0;
            cmd.roll       = 0.0;
            cmd.pitch      = 0.0;
            cmd.yaw        = 0.0;
            cmd.z          = 0.0;
            cmd.faster     = 0.0;
            cmd.slower     = 0.0;

            if (cmd.movement == Viewing)
            {
                cmd.movement = Stepping;
                cmd.motion   = Stop;
                std::cout << "[SPOT] Switched to STEPPING mode "
                          << "(vx | vy | w | z)" << std::endl;
            }
            else
            {
                cmd.movement = Viewing;
                cmd.motion   = Stop;
                std::cout << "[SPOT] Switched to VIEWING mode "
                          << "(roll | pitch | yaw | z)" << std::endl;
            }
        }
    }

    SpotCommand Spot::return_command()
    {
        return cmd;
    }
}