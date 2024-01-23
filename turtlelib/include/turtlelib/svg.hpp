#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Visualize 2D geometry in SVG format.


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
        /// \brief Default Constructor, intalizes the svg file
        BuildSVG();

        /// \brief print the svg string
        /// \returns the svg string
        std::string ExportSVG();

        /// \brief print the svg string to a file
        /// \param filename - the name of the file to write to
        void ExportSVG(std::string filename);

        /// \brief draw a point
        /// \param point - the point to draw
        /// \param color - the color of the point
        void Draw(turtlelib::Point2D point, std::string color);

        /// \brief draw an arrow
        /// \param head - the head of the arrow
        /// \param tail - the tail of the arrow
        void Draw(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color);

        /// \brief draw a coordinate frame
        /// \param T - the transform from the frame to the world
        /// \param label - the label for the frame
        void Draw(turtlelib::Transform2D T, std::string label);

    private: 
        std::string svg_file;

    };

}

#endif