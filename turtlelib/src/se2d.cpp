#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
# include <iostream>
# include <cmath>
# include <vector>

using namespace std;
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Twist2D & tw)
{

    os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Twist2D & tw)
{
// Assign a character to IS in order to get user input for options about format
        char c;
        is >> c;

        // If the character is not '['
        if (c != '[')
        {
            // 
            is.putback(c);

            // Read the Point
            is >> tw.omega >> tw.x >> tw.y >> c;
        }
        else
        {
            // Read the Point
            is >> tw.omega >> tw.x >> tw.y >> c;
        }

        return is;


}
/// \brief Create an identity transformation
turtlelib::Transform2D::Transform2D()
{

    // double theta = 0.0;
    // turtlelib::Vector2D trans;


    matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    
    matrix[0][0] = cos(theta);
    matrix[1][0] = sin(theta);
    matrix[0][1] = -sin(theta);
    matrix[1][1] = cos(theta);
    matrix[0][2] = trans.x;
    matrix[1][2] = trans.y;
    matrix[2][2] = 1.0;
    matrix[2][0] = 0.0; 
    matrix[2][1] = 0.0;

}

turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans)
{

    matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    matrix[0][0] = cos(theta);
    matrix[1][0] = sin(theta);
    matrix[0][1] = -sin(theta);
    matrix[1][1] = cos(theta);
    matrix[0][2] = trans.x;
    matrix[1][2] = trans.y;
    matrix[2][2] = 1.0;
    matrix[2][0] = 0.0; 
    matrix[2][1] = 0.0;
    
}

turtlelib::Transform2D::Transform2D(double radians)
{

    matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    matrix[0][0] = cos(radians);
    matrix[1][0] = sin(radians);
    matrix[0][1] = -sin(radians);
    matrix[1][1] = cos(radians);
    matrix[0][2] = trans.x;
    matrix[1][2] = trans.y;
    matrix[2][2] = 1.0;
    matrix[2][0] = 0.0; 
    matrix[2][1] = 0.0;
    
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    matrix[0][0] = cos(radians);
    matrix[1][0] = sin(radians);
    matrix[0][1] = -sin(radians);
    matrix[1][1] = cos(radians);
    matrix[0][2] = trans.x;
    matrix[1][2] = trans.y;
    matrix[2][2] = 1.0;
    matrix[2][0] = 0.0; 
    matrix[2][1] = 0.0;

}

turtlelib::Point2D turtlelib::Transform2D::operator()(turtlelib::Point2D p) const
{

    turtlelib::Point2D p2;
    p2.x = matrix[0][0] * p.x + matrix[0][1] * p.y + matrix[0][2];
    p2.y = matrix[1][0] * p.x + matrix[1][1] * p.y + matrix[1][2];
    return p2;

}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const

{
    turtlelib::Vector2D v2;
    v2.x = matrix[0][0] * v.x + matrix[0][1] * v.y + matrix[0][2];
    v2.y = matrix[1][0] * v.x + matrix[1][1] * v.y + matrix[1][2];
    return v2;


}
turtlelib::Twist2D turtlelib::Transform2D::operator()(turtlelib::Twist2D V) const
{
    turtlelib::Twist2D tw;

    tw.omega = V.omega; 
    tw.x = V.omega*matrix[1][2] + matrix[0][0] * V.x + matrix[0][1] * V.y;
    tw.y = -V.omega*matrix[0][2] + matrix[1][0] * V.x + matrix[1][1] * V.y;
    return tw;


}

turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
    turtlelib::Transform2D Tinv;
    Tinv.matrix[0][0] = matrix[0][0];
    Tinv.matrix[1][0] = matrix[0][1];
    Tinv.matrix[0][1] = matrix[1][0];
    Tinv.matrix[1][1] = matrix[1][1];
    Tinv.matrix[0][2] = -matrix[0][2]*Tinv.matrix[0][0] - matrix[1][2]*Tinv.matrix[0][1];
    Tinv.matrix[1][2] = -matrix[0][2]*Tinv.matrix[1][0] - matrix[1][2]*Tinv.matrix[1][1];

    return Tinv;

}

// turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const turtlelib::Transform2D & rhs)
// {



// }

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    turtlelib::Vector2D v;
    v.x = matrix[0][2];
    v.y = matrix[1][2];

    return v;
}

double turtlelib::Transform2D::rotation() const
{
    return acos(matrix[0][0]);

}

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Transform2D & tf)
{
    os << "deg: " << turtlelib::rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;

    return os;

}

// NOTE: ASK MATT ABOUT THIS
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Transform2D & tf)
{
    // turtlelib::Vector2D v;
    // double theta;
    is >> tf.theta >> tf.trans.x >> tf.trans.y;
    // tf = turtlelib::Transform2D(v, turtlelib::deg2rad(theta));

    return is;
}