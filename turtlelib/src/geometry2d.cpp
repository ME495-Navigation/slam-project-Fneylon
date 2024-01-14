#include "turtlelib/geometry2d.hpp"

    /// \brief wrap an angle to (-turtlelib::PI, turtlelib::PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-turtlelib::PI, turtlelib::PI]
 double turtlelib::normalize_angle(double rad)
    {
        // Find Number of Revolution Made by the Angle and the Remainder
        double rev = std::fmod(rad, 2 * turtlelib::PI);

        if (rev > turtlelib::PI)
        {
            rev -= 2 * turtlelib::PI;
        }
        else if (rev < -turtlelib::PI)
        {
            rev += 2 * turtlelib::PI;
        }
        else if (rev == -turtlelib::PI)
        {
            rev = turtlelib::PI;
        }

        return rev;

    }

    /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param p - the point to print
    std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Point2D & p)
    {
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }

        /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param p [out] - output vector
    /// HINT: See operator>> for Vector2D
    std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Point2D & p)
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
            is >> p.x >> p.y >> c;
        }
        else
        {
            // Read the Point
            is >> p.x >> p.y >> c;
        }

        return is;
    }

    turtlelib::Vector2D turtlelib::operator-(const turtlelib::Point2D & head, const turtlelib::Point2D & tail)
    {
        turtlelib::Vector2D v;
        v.x = head.x - tail.x;
        v.y = head.y - tail.y;
        return v;

    }

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    turtlelib::Point2D turtlelib::operator+(const turtlelib::Point2D & tail, const turtlelib::Vector2D & disp)
    {
        turtlelib::Point2D p;
        p.x = tail.x + disp.x;
        p.y = tail.y + disp.y;
        return p;

    }

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;

    }

    std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Vector2D & v)
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
            is >> v.x >> v.y >> c;
        }
        else
        {
            // Read the Point
            is >> v.x >> v.y >> c;
        }

        return is;

    }