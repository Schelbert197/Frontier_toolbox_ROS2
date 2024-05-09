#include <iostream>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"


#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

namespace turtlelib{
    TEST_CASE("vector stream insertion operator << Twist2D", "[transform]"){ // Srikanth, Schelbert
        Twist2D tw;
        tw.omega = 1.2;
        tw.x = 2.4;
        tw.y = 4.8;
        std::string str = "[1.2 2.4 4.8]";
        std::stringstream sstr;
        sstr << tw;
        REQUIRE(sstr.str() == str);
    }

    TEST_CASE("vector stream extraction operator >> Twist2D", "[transform]"){ // Srikanth, Schelbert 
        Twist2D tw;
        std::string str = "[1.2 2.4 4.8]";
        std::stringstream sstr;
        sstr << str;
        sstr >> tw;
        REQUIRE_THAT(tw.omega,Catch::Matchers::WithinAbs(1.2,1e-5));
        REQUIRE_THAT(tw.x,Catch::Matchers::WithinAbs(2.4,1e-5));
        REQUIRE_THAT(tw.y,Catch::Matchers::WithinAbs(4.8,1e-5));
    }

    TEST_CASE("rotation()","[transform]"){ // adapted from Shail, Dalal
        double test_rot {PI};
        Transform2D T_test(test_rot);
        REQUIRE(T_test.rotation() == PI);
    }

    TEST_CASE("translation()","[transform]"){ // Srikanth, Schelbert
        Vector2D test_vec = {4.0, 8.0};
        Transform2D T_test(test_vec);
        REQUIRE(T_test.translation().x == test_vec.x);
        REQUIRE(T_test.translation().y == test_vec.y);
    }

    TEST_CASE("operator()(Vector2D v)","[transform]"){ // Srikanth, Schelbert
        double test_rot = -PI/2.0;
        Vector2D test_vec = {1.0, 1.0};
        Transform2D T_ab{{test_vec}, test_rot};
        Vector2D v_b{1, 1};
        Vector2D v_a = T_ab(v_b);
        REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    }

    TEST_CASE("operator()(Point2D v)","[transform]"){ // Srikanth, Schelbert
        double test_rot = -PI/2.0;
        Vector2D test_vec = {1.0, 1.0};
        Transform2D T_ab{{test_vec}, test_rot};
        Point2D p_b{1, 1};
        Point2D p_a = T_ab(p_b);
        REQUIRE_THAT(p_a.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
        REQUIRE_THAT(p_a.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    TEST_CASE("operator()(Twist2D t)","[transform]"){ // Srikanth Schelbert
        double test_rot = PI/2.0;
        Vector2D test_vec = {0.0, 1.0};
        Transform2D T_ab{{test_vec}, test_rot};
        Twist2D T_b{1, 1, 1};
        Twist2D T_a = T_ab(T_b);
        REQUIRE_THAT(T_a.omega, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }

    TEST_CASE("inverse - inv()", "[transform]"){ // Srikanth, Schelbert
        double test_rot = -PI/2.0;
        double test_x = 1.0;
        double test_y = 1.0; // Just make this a vector
        Transform2D T_test{{test_x,test_y}, test_rot};
        Transform2D T_test_inv = T_test.inv();
        REQUIRE(T_test.inv().rotation() == -test_rot);
        REQUIRE_THAT(T_test_inv.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(T_test_inv.translation().y, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    }

    TEST_CASE("matrix mult operator - operator*=", "[transform]"){ // Srikanth, Schelbert
        Vector2D test_trans_ab = {1.0, 2.0};
        double test_rot_ab = 0.0;
        Vector2D test_trans_bc = {2.0, 3.0};
        double test_rot_bc = PI/2;
        Transform2D T_ab1 = {test_trans_ab, test_rot_ab};
        Transform2D T_ab2 = {test_trans_ab, test_rot_ab};
        Transform2D T_ab3 = {test_trans_ab, test_rot_ab}; // made 3 since fcn returns overwritten tf
        Transform2D T_bc = {test_trans_bc, test_rot_bc};
        REQUIRE_THAT((T_ab1*=T_bc).translation().x, Catch::Matchers::WithinAbs(3.0, 1e-5));
        REQUIRE_THAT((T_ab2*=T_bc).translation().y, Catch::Matchers::WithinAbs(5.0, 1e-5));
        REQUIRE_THAT((T_ab3*=T_bc).rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1e-5));
    }

    TEST_CASE( "output stream transform", "[transform]" ){ //Abhishek, Sankar
        Transform2D transform(Vector2D{2.3,3.1}, PI);
        std::stringstream os;
        os << transform;
        REQUIRE( os.str() == "deg: 180 x: 2.3 y: 3.1" );
    }

    TEST_CASE( "input stream transform", "[transform]" ){ //Abhishek, Sankar
        Transform2D transform_1,transform_2,transform_3;

        std::stringstream is_1,is_2,is_3;

        is_1 << "deg: 180 x: 2.3 y: 3.1"; //test type output of << operator
        is_1 >> transform_1;

        is_2 << "180 2.3 3.1"; //test type separated by spaces
        is_2 >> transform_2;

        is_3 << "180\n2.3\n3.1"; //test type separated by new line
        is_3 >> transform_3;

        REQUIRE_THAT(transform_1.rotation(), Catch::Matchers::WithinRel(PI));
        REQUIRE_THAT(transform_1.translation().x, Catch::Matchers::WithinRel(2.3));
        REQUIRE_THAT(transform_1.translation().y, Catch::Matchers::WithinRel(3.1));
        REQUIRE_THAT(transform_2.rotation(), Catch::Matchers::WithinRel(PI));
        REQUIRE_THAT(transform_2.translation().x, Catch::Matchers::WithinRel(2.3));
        REQUIRE_THAT(transform_2.translation().y, Catch::Matchers::WithinRel(3.1));
        REQUIRE_THAT(transform_3.rotation(), Catch::Matchers::WithinRel(PI));
        REQUIRE_THAT(transform_3.translation().x, Catch::Matchers::WithinRel(2.3));
        REQUIRE_THAT(transform_3.translation().y, Catch::Matchers::WithinRel(3.1));
    }

    TEST_CASE( "transform multiplication", "[transform]" ){ //Abhishek, Sankar
        Transform2D transform_1(Vector2D{1.0,1.0}, 0.0);
        Transform2D transform_2(Vector2D{2.0,2.0}, PI/2.0);
        Transform2D resulting_transform;

        resulting_transform = transform_1*transform_2;

        REQUIRE_THAT(resulting_transform.rotation(), Catch::Matchers::WithinRel(PI/2.0));
        REQUIRE_THAT(resulting_transform.translation().x, Catch::Matchers::WithinRel(3.0)); //check
        REQUIRE_THAT(resulting_transform.translation().y, Catch::Matchers::WithinRel(3.0));
    }

    TEST_CASE("integrate_twist()", "[transform]"){ // Srikanth Schelbert
        // Pure traslation
        Twist2D t1 = {0.0, 1.2, 3.4};
        Transform2D T1 = turtlelib::integrate_twist(t1);
        REQUIRE_THAT(T1.translation().x, Catch::Matchers::WithinAbs(1.2, 1e-5));
        REQUIRE_THAT(T1.translation().y, Catch::Matchers::WithinAbs(3.4, 1e-5));
        // Pure rotation
        Twist2D t2 = {-PI, 0.0, 0.0};
        Transform2D T2 = turtlelib::integrate_twist(t2);
        REQUIRE_THAT(T2.rotation(), Catch::Matchers::WithinAbs(-PI, 1e-5));
        // Rotation and Traslation
        Twist2D t3 = {-1.24, -2.15, -2.92};
        Transform2D T3 = turtlelib::integrate_twist(t3);
        REQUIRE_THAT(T3.translation().x, Catch::Matchers::WithinAbs(-3.229863264722, 1e-5));
        REQUIRE_THAT(T3.translation().y, Catch::Matchers::WithinAbs(-1.05645265317421, 1e-5));
        REQUIRE_THAT(T3.rotation(), Catch::Matchers::WithinAbs(-1.24, 1e-5));
    }
}