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
        if (c == '[')
        {
            // 
            

            // Read the Point
            is >> tw.omega >> tw.x >> tw.y >> c;
        }
        else
        {
            // Read the Point
            is.putback(c);
            is >> tw.omega >> tw.x >> tw.y;
        }

        return is;


}
/// \brief Create an identity transformation
turtlelib::Transform2D::Transform2D()
{
    vector.x = 0.0;
    vector.y = 0.0;
    theta = 0.0;

}

turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans)
{
    vector.x = trans.x;
    vector.y = trans.y;
    theta = 0.0;

    
}

turtlelib::Transform2D::Transform2D(double radians)
{
    vector.x = 0.0;
    vector.y = 0.0;
    theta = radians;
    
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    vector.x = trans.x;
    vector.y = trans.y;
    theta = radians;

}

turtlelib::Point2D turtlelib::Transform2D::operator()(turtlelib::Point2D p) const
{

    turtlelib::Point2D p2;
    p2.x = cos(theta) * p.x - p.y*sin(theta) + vector.x;
    p2.y = p.x*sin(theta)+ p.y*cos(theta) + vector.y;

    return p2;

}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const

{
    turtlelib::Vector2D v2;
    v2.x = cos(theta) * v.x + v.y*(-sin(theta));
    v2.y = v.x*sin(theta)+ v.y*cos(theta);
    return v2;


}
turtlelib::Twist2D turtlelib::Transform2D::operator()(turtlelib::Twist2D v) const
{
    turtlelib::Twist2D tw;

    tw.omega = v.omega; 
    tw.x = v.omega*vector.y + cos(theta)*v.x + (-sin(theta))*v.y;
    tw.y = -v.omega*vector.x + sin(theta)*v.x + cos(theta)*v.y;
    return tw;

}

turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
    turtlelib::Transform2D Tinv;
    Tinv.vector.x = -vector.x*cos(theta) - vector.y*sin(theta);
    Tinv.vector.y = vector.x*sin(theta) - cos(theta)*vector.y;
    Tinv.theta = -theta;
    

    return Tinv;

}

turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const turtlelib::Transform2D & rhs)
{
    // turtlelib::Transform2D T;
    vector.x = vector.x + cos(theta)*rhs.vector.x -sin(theta)*rhs.vector.y;
    vector.y = vector.y + sin(theta)*rhs.vector.x + cos(theta)*rhs.vector.y;
    // T.vector.x = vector.x + cos(theta)*rhs.vector.x -sin(theta)*rhs.vector.y;
    // T.vector.y = vector.y + sin(theta)*rhs.vector.x + cos(theta)*rhs.vector.y;
    theta = theta + rhs.theta;
    return *this;

}

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    turtlelib::Vector2D v;
    v.x = vector.x;
    v.y =  vector.y;

    return v;
}

double turtlelib::Transform2D::rotation() const
{
    return theta;

}

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Transform2D & tf)
{
    os << "deg: " << turtlelib::rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;

    return os;

}

std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Transform2D & tf)
{
    turtlelib::Vector2D v;
    double gamma;
    is >> gamma >> v.x >> v.y;
    gamma = turtlelib::deg2rad(gamma);
    tf = turtlelib::Transform2D(v, gamma);

    return is;
}

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D & rhs)
{

    lhs*=rhs;
    return lhs;


}