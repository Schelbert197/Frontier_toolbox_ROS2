#include <iostream>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

// using turtlelib::Transform2D;

namespace turtlelib{
    /// \brief print the Twist2D in the format [w x y]
    /// \param os [in/out] the ostream to write to
    /// \param tw the twist to output
    /// \returns the ostream os  with the twist data inserted
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    /// \brief read the Twist2D in the format [w x y] or as w x y
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw){
        char c = is.peek(); // examine the next character without extracting it

        if (c == '['){
            is.get(); // remove the '[' character from the stream
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
            is.get(); // remove the ']' character from the stream
        }
        else{
            is>>tw.omega >> tw.x >> tw.y;
        }

        is.ignore(100,'\n'); //read for 100 char or until new line
        return is;
    }

    Transform2D::Transform2D() : translate{0.0, 0.0}, rotate(0.0){}

    Transform2D::Transform2D(Vector2D trans) : translate(trans), rotate(0.0){}

    Transform2D::Transform2D(double radians) : translate{0.0, 0.0}, rotate(radians){}

    Transform2D::Transform2D(Vector2D trans, double radians) : translate(trans), rotate(radians){}

    Point2D Transform2D::operator()(Point2D p) const{ // check if this is right
        return Point2D{p.x*cos(rotate) - p.y*sin(rotate) + translate.x, p.x*sin(rotate) + p.y*cos(rotate) + translate.y}; //FIX THIS
    }

    Vector2D Transform2D::operator()(Vector2D v) const{ // check if this is right
        return Vector2D{v.x*cos(rotate) - v.y*sin(rotate), v.x*sin(rotate) + v.y*cos(rotate)};
    }

    Twist2D Transform2D::operator()(Twist2D t) const{
        Twist2D newTwist;
        newTwist.omega = t.omega;
        newTwist.x = t.omega*translate.y + t.x*cos(rotate) - t.y*sin(rotate);
        newTwist.y = -t.omega*translate.x + t.x*sin(rotate) + t.y*cos(rotate);
        return newTwist;
    }

    Transform2D Transform2D::inv() const{
        Transform2D trans2D;
        trans2D.translate.x = -translate.x*cos(rotate)-translate.y*sin(rotate);
        trans2D.translate.y = -translate.y*cos(rotate)+translate.x*sin(rotate);
        trans2D.rotate = -rotate;
        return trans2D;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        translate.x = rhs.translate.x*cos(rotate) - rhs.translate.y*sin(rotate) + translate.x;
        translate.y = rhs.translate.x*sin(rotate) + rhs.translate.y*cos(rotate) + translate.y;
        rotate += rhs.rotate;
        return *this;
    }

    Vector2D Transform2D::translation() const{
        return translate;
    }

    double Transform2D::rotation() const{
        return rotate;
    }

    Transform2D integrate_twist(Twist2D tw){
        if (tw.omega == 0) {
            double x = tw.x;
            double y = tw.y;
            Transform2D Tbb_prime(Vector2D{x, y}); // Pure Translation
            return Tbb_prime;
        } 
        else {
            double x = tw.y / tw.omega;
            double y = -tw.x / tw.omega;
            Transform2D Tsb(Vector2D{x, y}); // Translation & Rotation or Pure Rotation
            Transform2D Tss_prime(tw.omega); // Pure rotation
            Transform2D Tb_prime_s_prime;
            Transform2D Ts_prime_b_prime;
            Transform2D Tbs;
            Transform2D Tbb_prime;
            Tbs = Tsb.inv();
            Tb_prime_s_prime = Tbs;
            Ts_prime_b_prime = Tb_prime_s_prime.inv();
            Tbb_prime = Tbs * Tss_prime * Ts_prime_b_prime;
            return Tbb_prime;
        }
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        return os << "deg: " << rad2deg(tf.rotate) << " " << "x: " << tf.translate.x << " " << "y: " << tf.translate.y;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        std::string str, str2, str3;
        double rot = 0.0;
        Vector2D tran{0.0,0.0};

        char c = is.peek(); // examine the next character without extracting it

        if (c == 'd'){
            // remove the 'deg: ' from the stream
            is >> str; // deg:
            is >> rot; // remove 'x: ' from the stream
            is >> str2; // x:
            is >> tran.x; // remove 'y: ' from the stream
            is >> str3; // y:
            is >> tran.y; 
        }
        else{
             is >> rot >> tran.x >> tran.y; // Extract values from is buffer
        }
        is.ignore(100,'\n'); //read for 100 char or until new line
        // Change deg input to radians for calculations
        rot = deg2rad(rot);
        // Use constructer with values extracted from the is stream
        tf = Transform2D{tran, rot}; // Use reference to Transform2D objct tf to save input values
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs; //return the lhs using the T2D operator we made
    }

}