#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib{
    // Constructor Definitions
    DiffDrive::DiffDrive()
    : wheel_radius(turtlebot3_wheel_radius),
    wheel_track(turtlebot3_wheel_track),
    q{0.0, 0.0, 0.0},
    wheel_position{0.0, 0.0} {}

    DiffDrive::DiffDrive(double radius, double track)
    : wheel_radius(radius),
    wheel_track(track),
    q{0.0, 0.0, 0.0},
    wheel_position{0.0, 0.0} {}

    DiffDrive::DiffDrive(double radius, double track, RobotConfiguration robot_config)
    : wheel_radius(radius),
    wheel_track(track),
    q{robot_config},
    wheel_position{0.0, 0.0} {}

    RobotConfiguration DiffDrive::configuration() const{
        return q;
    }

    void DiffDrive::set_configuration(RobotConfiguration q_new){
        q.x = q_new.x;
        q.y = q_new.y;
        q.theta = q_new.theta;
    }

    Twist2D DiffDrive::Twist(WheelPositions delta_wheel_positions){
        wheel_position.left = wheel_position.left + delta_wheel_positions.left;
        wheel_position.right = wheel_position.right + delta_wheel_positions.right;

        WheelVelocities wheel_vel{}; // eqn 2
        wheel_vel.left = delta_wheel_positions.left;
        wheel_vel.right = delta_wheel_positions.right;

        Twist2D tw_b_bprime; // eqn 3
        tw_b_bprime.omega = (wheel_radius / wheel_track) * (-wheel_vel.left + wheel_vel.right);
        tw_b_bprime.x = (wheel_radius / 2) * (wheel_vel.left + wheel_vel.right);
        tw_b_bprime.y = 0;
        return tw_b_bprime;
    }

    void DiffDrive::ForwardKinematics(WheelPositions delta_wheel_positions){
        Twist2D twist_bbp = DiffDrive::Twist(delta_wheel_positions); // Get b_bprime twist
        // Transformation matrix between current and end position
        auto T_b_bprime = integrate_twist(twist_bbp); // eqn 3

        Transform2D Twb(Vector2D{q.x, q.y}, q.theta); // Current position to world: eqn 4
        Transform2D T_w_bprime; // End position to world
        T_w_bprime = Twb * T_b_bprime; // eqn 5

        // New configuration
        q.x = T_w_bprime.translation().x;
        q.y = T_w_bprime.translation().y;
        q.theta = normalize_angle(T_w_bprime.rotation());
    }

    WheelVelocities DiffDrive::InverseKinematics(const Twist2D twist){
        WheelVelocities wheel_vel;
        if (twist.y != 0.0) {
            throw std::logic_error("Twist cannot be accomplished without the wheels slipping!");
        } else { // equation set 1
            wheel_vel.left = (1 / wheel_radius) * (-(wheel_track / 2) * twist.omega + twist.x);
            wheel_vel.right = (1 / wheel_radius) * ((wheel_track / 2) * twist.omega + twist.x);
        }
        return wheel_vel;
    }

}