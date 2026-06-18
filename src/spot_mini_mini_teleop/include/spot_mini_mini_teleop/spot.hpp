#ifndef SPOT_MINI_MINI_TELEOP__SPOT_HPP_
#define SPOT_MINI_MINI_TELEOP__SPOT_HPP_

/// \file
/// \brief Spot library — high-level motion commands for Spot Mini Mini
/// Converted from ROS1 to ROS2 — only change is removing ros/ros.h
/// All logic is identical to original

#include <cmath>
#include <vector>

namespace spot
{
    /// \brief approximately compare two floating-point numbers
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-1)
    {
        return fabs(d1 - d2) < epsilon;
    }

    /// \brief Motion state — is the robot moving or stopped
    enum Motion {Go, Stop};

    /// \brief Movement mode — stepping (walking) or viewing (body pose)
    enum Movement {Stepping, Viewing};

    /// \brief Full command struct sent from state machine to policy runner
    struct SpotCommand
    {
        Motion   motion   = Stop;
        Movement movement = Viewing;
        double x_velocity = 0.0;  // forward/backward velocity
        double y_velocity = 0.0;  // lateral velocity
        double rate       = 0.0;  // yaw rate
        double roll       = 0.0;  // body roll  (Viewing mode)
        double pitch      = 0.0;  // body pitch (Viewing mode)
        double yaw        = 0.0;  // body yaw   (Viewing mode)
        double z          = 0.0;  // body height
        double faster     = 0.0;  // increase clearance height
        double slower     = 0.0;  // decrease clearance height
    };

    /// \brief Spot class — converts joystick Twist into SpotCommand
    class Spot
    {
    public:
        Spot();

        /// \brief Update internal command from joystick Twist values
        /// \param vx   linear x  (step length OR pitch depending on mode)
        /// \param vy   linear y  (lateral fraction OR roll depending on mode)
        /// \param z    height control
        /// \param w    angular z (yaw rate OR yaw depending on mode)
        /// \param wx   right bumper axis (faster/clearance)
        /// \param wy   left bumper axis  (slower/clearance)
        void update_command(const double & vx, const double & vy, const double & z,
                            const double & w,  const double & wx, const double & wy);

        /// \brief Switch between Stepping and Viewing movement modes
        /// Resets all velocities to zero before switching
        void switch_movement();

        /// \brief Return current SpotCommand for external use
        SpotCommand return_command();

    private:
        SpotCommand cmd;
    };
}

#endif  // SPOT_MINI_MINI_TELEOP__SPOT_HPP_
