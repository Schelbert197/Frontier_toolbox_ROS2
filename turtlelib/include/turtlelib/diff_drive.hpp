#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of a differential drive turtlebot3.

#include<iosfwd>
#include<cmath>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief the wheel radius of the turtlebot3 burger
    constexpr double turtlebot3_wheel_radius=0.033;
    /// \brief the wheel track of the turtlebot3 burger
    constexpr double turtlebot3_wheel_track=0.16;

    /// \brief Position of both wheels
    struct WheelPositions
    {
        /// \brief left rotational wheel position in radians
        double left = 0.0;
        /// \brief right rotational wheel position in radians
        double right = 0.0;
    };

    /// \brief Velocity of both wheels
    struct WheelVelocities
    {
        /// \brief left wheel velocity
        double left = 0.0;
        /// \brief right wheel velocity
        double right = 0.0;
    };

    /// \brief Robot configuration
    struct RobotConfiguration
    {
        /// \brief the x coordinate
        double x = 0.0;
        /// \brief the y coordinate
        double y = 0.0;
        /// \brief the orientation angle
        double theta = 0.0;
    };

    /// \brief Kinematics of a differential drive robot.
    class DiffDrive
    {
    private:
        /// \brief radius of wheels
        double wheel_radius;
        /// \brief distance between the wheels
        double wheel_track;
        /// \brief robot configuration x, y, theta
        RobotConfiguration q;
        /// \brief position of both wheels
        WheelPositions wheel_position;

    public:
        // ### BEGIN CITATION [6] ###
        /// \brief start at origin and default to turtlebo3 burger specifications
        DiffDrive();

        /// \brief set wheel specifications and start at origin
        /// \param radius - radius of wheels
        /// \param track - distance between the wheels
        DiffDrive(double radius, double track);

        /// \brief set start configuration and wheel specifications
        /// \param radius - wheel radius
        /// \param track - distance between wheels
        /// \param robot_config - robot configuration x, y, theta
        DiffDrive(double radius, double track, RobotConfiguration robot_config);

        /// \brief the robots current configuration
        /// \return robots current configuration
        RobotConfiguration configuration() const;

        /// \brief set robots current configuration
        void set_configuration(RobotConfiguration q_new);

        /// \brief generates a twist from wheel positions
        /// \param delta_wheel_positions - change in wheel positions
        /// \return body twist
        Twist2D Twist(WheelPositions delta_wheel_positions);

        /// \brief Calculates the forward kinematics from the new wheel positions
        /// \param delta_wheel_positions - change in wheel positions
        void ForwardKinematics(WheelPositions delta_wheel_positions);

        /// \brief Calculates the inverse kinematics from a body twist
        /// \param twist - the body twist to preform inverse kinematics on
        /// \return wheel velocities required for the twist
        WheelVelocities InverseKinematics(const Twist2D twist);
        // ### END CITATION [6] ###
    };

}

#endif