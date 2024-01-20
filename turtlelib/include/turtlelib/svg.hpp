#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include <cstdlib>
#include <iostream>
#include <cmath>
#include<vector>

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib
{
    class BuildSVG
    {

    public:
        // Initalizes the Default Constructor i.e. starts to build the file.
        // The Default Constructor is what happens when you create an object without any parameters.
        // This is the same as __init__ in python.
        BuildSVG();

        std::string ExportSVG();
        void ExportSVG(std::string filename);
        void Draw(turtlelib::Point2D point, std::string color);
        void Draw(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color);
        void Draw(turtlelib::Transform2D T, std::string label);

    private: 
        std::string svg_file;

    };

}

#endif