#include "turtlelib/geometry2d.hpp"

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


    std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Point2D & p)
    {
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }


    std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Point2D & p)
    {
        // Assign a character to IS in order to get user input for options about format
        char c;
        is >> c;

        // If the character is not '['
        if (c == '[')
        {
            // Read the Point
            is >> p.x >> p.y >> c;
        }
        else
        {
            // Read the Point
            is.putback(c);
            is >> p.x >> p.y;
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


    turtlelib::Point2D turtlelib::operator+(const turtlelib::Point2D & tail, const turtlelib::Vector2D & disp)
    {
        turtlelib::Point2D p;
        p.x = tail.x + disp.x;
        p.y = tail.y + disp.y;
        return p;

    }


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
        if (c == '[')
        {
            // 
           

            // Read the Point
            is >> v.x >> v.y >> c;
        }
        else
        {
            // Read the Point
             is.putback(c);
            is >> v.x >> v.y;
        }

        return is;

    }

    turtlelib::Vector2D turtlelib::operator+=(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
    {
        // turtlelib::Vector2D v;
        v1.x = v1.x + v2.x;
        v1.y = v1.y + v2.y;
        return v1;

    }

    turtlelib::Vector2D & turtlelib::operator+(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
    {
        v1+=v2;

        return v1;
    }

    turtlelib::Vector2D turtlelib::operator-=(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
    {
        // turtlelib::Vector2D v;
        v1.x = v1.x - v2.x;
        v1.y = v1.y - v2.y;
        return v1;

    }

    turtlelib::Vector2D & turtlelib::operator-(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
    {
        v1-=v2;
        return v1;

    }

    turtlelib::Vector2D & turtlelib::operator*(turtlelib::Vector2D & v, double mag)
    {
        v*=mag;
        return v;

    }

    turtlelib::Vector2D turtlelib::operator*=(turtlelib::Vector2D & v, double mag)
    {

        v.x = v.x * mag;
        v.y = v.y * mag;
        return v;
    }

    double turtlelib::dot(const turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
    {
        double dot = v1.x * v2.x + v1.y * v2.y;
        return dot;

    }

    double turtlelib::magnitude(const turtlelib::Vector2D & v)
    {
        double mag = std::sqrt(v.x * v.x + v.y * v.y);
        return mag;

    }

    double turtlelib::angle(const turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
    {
        // Begin citation [4]
        double angle = std::acos(turtlelib::dot(v1, v2) / (turtlelib::magnitude(v1) * turtlelib::magnitude(v2)));
        //end citation [4]
        return angle;

    }

    // turtlelib::Vector2D turtlelib::operator*(const turtlelib::Vector2D & v, double mag)
    // {
    //     turtlelib::Vector2D v1;
    //     v1.x = v.x * mag;
    //     v1.y = v.y * mag;
    //     return v1;

    // }

    turtlelib::Vector2D turtlelib::normalize(const turtlelib::Vector2D & v)
    {
        double mag = std::sqrt(v.x * v.x + v.y * v.y);
        turtlelib::Vector2D v1;
        v1.x = v.x * (1.0 / mag);
        v1.y = v.y * (1.0 / mag);
        return v1;

    }

