#include <iostream>
#include <cmath>
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace turtlelib{
    TEST_CASE("ForwardKinematics(), Pure rotation", "[diff_drive]"){
    DiffDrive turtlebot(1.0, 2.0);
    WheelPositions rot{-PI, PI};
    turtlebot.ForwardKinematics(rot);
    REQUIRE_THAT(turtlebot.configuration().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(turtlebot.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(turtlebot.configuration().theta, Catch::Matchers::WithinAbs(PI, 1.0e-5));
    }

    TEST_CASE("ForwardKinematics(), Pure translation", "[diff_drive]"){
    DiffDrive turtlebot(1.0, 2.0);
    WheelPositions fwd{PI, PI};
    turtlebot.ForwardKinematics(fwd);
    REQUIRE_THAT(turtlebot.configuration().x, Catch::Matchers::WithinAbs(PI, 1.0e-5));
    REQUIRE_THAT(turtlebot.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(turtlebot.configuration().theta, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    }

    TEST_CASE("InverseKinematics(), Pure translation", "[diff_drive]"){
    DiffDrive turtlebot(1.0, 2.0);
    Twist2D fwd{0,1,0};
    WheelVelocities wheels = turtlebot.InverseKinematics(fwd);
    REQUIRE_THAT(wheels.left, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(wheels.right, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    }

    TEST_CASE("InverseKinematics(), Pure rotation", "[diff_drive]"){
    DiffDrive turtlebot(1.0, 2.0);
    Twist2D rot{PI,0,0};
    WheelVelocities wheels = turtlebot.InverseKinematics(rot);
    REQUIRE_THAT(wheels.left, Catch::Matchers::WithinAbs(-PI, 1.0e-5));
    REQUIRE_THAT(wheels.right, Catch::Matchers::WithinAbs(PI, 1.0e-5));
    }

    // ### BEGIN CITATION [5] ###
    // Idea to make circle from Abhishek Sankar
    TEST_CASE("InverseKinematics() and ForwardKinematics(), Rotation and Translation", "[diff_drive]"){
    DiffDrive turtlebot(1.0, 2.0);
    Twist2D circle{2*PI, 1, 0};
    WheelVelocities turtlebot_wheel = turtlebot.InverseKinematics(circle);
    WheelPositions both{turtlebot_wheel.left, turtlebot_wheel.right};
    turtlebot.ForwardKinematics(both); // should make a big circle
    REQUIRE_THAT(turtlebot.configuration().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(turtlebot.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(turtlebot.configuration().theta, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    }
    // ### END CITATION [5] ###
    
    TEST_CASE("InverseKinematics(), Impossible twist", "[diff_drive]"){
    DiffDrive turtlebot(1.0,4.0);
    Twist2D slide{0, 0, 1};
    REQUIRE_THROWS_AS(turtlebot.InverseKinematics(slide), std::logic_error);
    }
}