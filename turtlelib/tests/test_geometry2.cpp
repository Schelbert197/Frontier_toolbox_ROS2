#include <iostream>
#include <cmath>
#include "turtlelib/geometry2d.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

// using turtlelib::Vector2D;
// using turtlelib::Point2D;
using Catch::Matchers::WithinAbs;
namespace turtlelib{
    TEST_CASE("operator +", "[geometry]"){ // Srikanth, Schelbert
        Point2D p1 = {1.5, 2.0};
        Vector2D v2 = {2.0, -0.5};
        REQUIRE_THAT((p1+v2).x, Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT((p1+v2).y, Catch::Matchers::WithinAbs(1.5, 1e-5));
    }

    TEST_CASE("operator -", "[geometry]"){ // Srikanth, Schelbert
        Point2D p1 = {1.5, 2.0};
        Point2D p2 = {2.0, -0.5};
        REQUIRE_THAT((p1-p2).x, Catch::Matchers::WithinAbs(-0.5, 1e-5));
        REQUIRE_THAT((p1-p2).y, Catch::Matchers::WithinAbs(2.5, 1e-5));
    }

    TEST_CASE("normalize angle", "[geometry]"){ // Srikanth, Schelbert
        REQUIRE_THAT(normalize_angle(PI),Catch::Matchers::WithinAbs(PI, 1e-5));
        REQUIRE_THAT(normalize_angle(-PI),Catch::Matchers::WithinAbs(PI, 1e-5));
        REQUIRE_THAT(normalize_angle(0),Catch::Matchers::WithinAbs(0, 1e-5));
        REQUIRE_THAT(normalize_angle(-PI/4),Catch::Matchers::WithinAbs(-PI/4, 1e-5));
        REQUIRE_THAT(normalize_angle(-2.5*PI),Catch::Matchers::WithinAbs(-0.5*PI, 1e-5));
        REQUIRE_THAT(normalize_angle(1.5*PI),Catch::Matchers::WithinAbs(-0.5*PI, 1e-5));
    }

    TEST_CASE("Vector2D operator +=", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0}; // vector will be rewritten twice
        REQUIRE_THAT((v1+=v1).x, Catch::Matchers::WithinAbs(3.0, 1e-5));
        REQUIRE_THAT((v1+=v1).y, Catch::Matchers::WithinAbs(8.0, 1e-5));
    }

    TEST_CASE("Vector2D operator -=", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0}; // vector will be rewritten
        Vector2D v2 = {8.0, 3.0};
        REQUIRE_THAT((v1-=v2).x, Catch::Matchers::WithinAbs(-6.5, 1e-5));
        REQUIRE_THAT((v1-=v2).y, Catch::Matchers::WithinAbs(-4.0, 1e-5));
    }

    TEST_CASE("Vector2D operator *=", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0};
        Vector2D v2 = {1.5, 2.0}; // vector will be rewritten
        double s = 3.0;
        REQUIRE_THAT((v1*=s).x, Catch::Matchers::WithinAbs(4.5, 1e-5));
        REQUIRE_THAT((v2*=s).y, Catch::Matchers::WithinAbs(6.0, 1e-5));
    }

    TEST_CASE("Vector2D operator +", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0};
        Vector2D v2 = {2.0, -0.5};
        REQUIRE_THAT((v1+v2).x, Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT((v1+v2).y, Catch::Matchers::WithinAbs(1.5, 1e-5));
    }

    TEST_CASE("Vector2D operator -", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0};
        Vector2D v2 = {2.0, -0.5};
        REQUIRE_THAT((v1-v2).x, Catch::Matchers::WithinAbs(-0.5, 1e-5));
        REQUIRE_THAT((v1-v2).y, Catch::Matchers::WithinAbs(2.5, 1e-5));
    }

    TEST_CASE("Vector2D operator *", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0};
        double s = 2.0;
        REQUIRE_THAT((v1*s).x, Catch::Matchers::WithinAbs(3.0, 1e-5));
        REQUIRE_THAT((v1*s).y, Catch::Matchers::WithinAbs(4.0, 1e-5));
    }

    TEST_CASE("Dot two vectors", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.5, 2.0};
        Vector2D v2 = {2.0, -0.5};
        double dot = turtlelib::dot(v1,v2);
        REQUIRE_THAT(dot, Catch::Matchers::WithinAbs(2.0, 1e-5));
    }

    TEST_CASE("Magnitude of a vector", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {3.0, 4.0};
        double mag = turtlelib::magnitude(v1);
        REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(5.0, 1e-5));
    }

    TEST_CASE("Angle between two vectors", "[geometry]"){ // Srikanth, Schelbert
        Vector2D v1 = {1.0, 1.0};
        Vector2D v2 = {1.0, -1.0};
        double angle = turtlelib::angle(v1,v2);
        REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(-PI/2, 1e-5));
    }

    TEST_CASE("vector stream insertion operator <<", "[geometry]"){ // Srikanth, Schelbert
        Vector2D vec;
        vec.x = 1.2;
        vec.y = 2.4;
        std::string str = "[1.2 2.4]";
        std::stringstream sstr;
        sstr << vec;
        REQUIRE(sstr.str() == str);
    }

    TEST_CASE("vector stream extraction operator >>", "[geometry]"){ // Srikanth, Schelbert
        Vector2D vec;
        std::string str = "[1.2 2.4]";
        std::stringstream sstr;
        sstr << str;
        sstr >> vec;
        REQUIRE_THAT(vec.x,Catch::Matchers::WithinAbs(1.2,1e-5));
    }

    TEST_CASE("point stream insertion operator <<", "[geometry]"){ // Srikanth, Schelbert
        Point2D point;
        point.x = 1.2;
        point.y = 2.4;
        std::string str = "[1.2 2.4]";
        std::stringstream sstr;
        sstr << point;
        REQUIRE(sstr.str() == str);
    }

    TEST_CASE("point stream extraction operator >>", "[geometry]"){ // Srikanth, Schelbert
        Point2D point;
        std::string str = "[1.2 2.4]";
        std::stringstream sstr;
        sstr << str;
        sstr >> point;
        REQUIRE_THAT(point.x,Catch::Matchers::WithinAbs(1.2,1e-5));
    }

    TEST_CASE("vector angle calculator", "[geometry]"){
        Vector2D v1 = {1.0, 1.0};
        Vector2D v2 = {-1.0, 1.0};
        Vector2D v3 = {1.0, 0.0};
        Vector2D v4 = {0.866, -0.5};
        double angle1 = angle(v1, v2);
        double angle2 = angle(v3, v4);
        REQUIRE_THAT(angle1, Catch::Matchers::WithinAbs(PI/2, 1e-5));
        REQUIRE_THAT(angle2, Catch::Matchers::WithinAbs(-PI/6, 1e-4));
    }
}
