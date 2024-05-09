#include <iostream>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
// using namespace turtlelib;

namespace turtlelib{
    /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param p - the point to print
    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        return os << "[" << p.x << " " << p.y << "]";
    }

    /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param p [out] - output vector
    /// HINT: See operator>> for Vector2D
    std::istream & operator>>(std::istream & is, Point2D & p){
        char c = is.peek(); // examine the next character without extracting it

        if (c == '[')
        {
            is.get(); // remove the '[' character from the stream
            is >> p.x;
            is >> p.y;
            is.get(); // remove the ']' character from the stream
        }
        else
        {
            is >> p.x >> p.y;
        }

        is.ignore(100,'\n'); //read for 100 char or until new line
        return is;

    }

    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad){
        // double wrapped = fmod(rad,2*PI);
        // if (wrapped > PI){ //
        //     wrapped = (wrapped - PI) - PI;
        // }
        // else if (wrapped <= -PI){
        //     wrapped = PI - (wrapped - PI);
        // }
        double wrapped = atan2(sin(rad),cos(rad));
        if (abs(wrapped + 3.1415926536) < 0.00001){
            wrapped += 2*PI;
        }
        return wrapped;
    }

    Vector2D normalize_vector(Vector2D vec){
        const auto magnitude = sqrt((vec.x * vec.x) + (vec.y * vec.y));
        return {vec.x / magnitude, vec.y / magnitude};
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs){
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs){
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & rhs){
        x *= rhs;
        y *= rhs;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs){
        return lhs += rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs){
        return lhs -= rhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs){
        return lhs *= rhs;
    }

    Vector2D operator*(const double & lhs, Vector2D rhs){
        return rhs *= lhs;
    }

    double dot(Vector2D v1, Vector2D v2){
        return (double)((v1.x * v2.x) + (v1.y * v2.y));
    }

    double magnitude(Vector2D v){
        return (double)std::sqrt((v.x * v.x) + (v.y * v.y));
    }

    double angle(Vector2D v1, Vector2D v2){ // atan2 range -pi, pi
    // ############################## Begin_Citation [2] ##############################
        return atan2((v1.x * v2.y) - (v1.y * v2.x), (v1.x * v2.x) + (v1.y * v2.y));
    // ############################## End_Citation [2]   ##############################
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){
        return Vector2D{head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        char c = is.peek(); // examine the next character without extracting it

        if (c == '['){
            is.get(); // remove the '[' character from the stream
            is >> v.x;
            is >> v.y;
            is.get(); // remove the ']' character from the stream
        }
        else{
            is >> v.x >> v.y;
        }
        
        is.ignore(100,'\n'); //read for 100 char or until new line
        return is;
    }
}