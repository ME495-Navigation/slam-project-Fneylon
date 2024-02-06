#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
# include <iostream>
# include <cstdlib>
# include <iostream>
# include <cmath>
# include <vector>
# include <fstream>

turtlelib::BuildSVG::BuildSVG()
{
    svg_file = "\n<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    svg_file += "\n<defs>";
    svg_file += "\n\t<marker";
    svg_file += "\n\t\tstyle=\"overflow:visible\"";
    svg_file += "\n\t\tid=\"Arrow1Sstart\"";
    svg_file += "\n\t\trefX=\"0.0\"";
    svg_file += "\n\t\trefY=\"0.0\"";
    svg_file += "\n\t\torient=\"auto\">";
    svg_file += "\n\t\t\t<path";
    svg_file += "\n\t\t\t\ttransform=\"scale(0.2) translate(6,0)\"";
    svg_file += "\n\t\t\t\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"";
    svg_file += "\n\t\t\t\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"";
    svg_file += "\n\t\t\t/>";
    svg_file += "\n\t</marker>";
    svg_file += "\n</defs>";
}

std::string turtlelib::BuildSVG::ExportSVG()
{
    svg_file += "\n</svg>";
    return svg_file;
}

void turtlelib::BuildSVG::ExportSVG(std::string filename)
{
    svg_file += "\n</svg>";
    std::ofstream file;
    file.open(filename);
    file << svg_file;
    file.close();
}

void turtlelib::BuildSVG::Draw(turtlelib::Point2D point, std::string color)
{
    double cx; // unitialized variables
    double cy;
    cx = 96.0*point.x + 0.5*8.5*96.0; // magic numbers should be constexpr somewhere
    cy = -96.0*point.y + 0.5*11.0*96.0;
    svg_file += "\n<circle ";
    svg_file += "cx=\"" + std::to_string(cx) + "\"";
    svg_file += " cy=\"" + std::to_string(cy) + "\"";
    svg_file += " r=\"3.0\"";
    svg_file += " fill=\"" + color + "\" stroke=\"" + color + "\" stroke-width=\"1.0\"/>";

}

void turtlelib::BuildSVG::Draw(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color)
{

    double x1;// unitialized varaibles
    double y1;
    double x2;
    double y2;

    x1 = 96.0*head.x + 0.5*8.5*96.0;
    y1 = -96.0*head.y + 0.5*11.0*96.0;
    x2 = 96.0*tail.x + 0.5*8.5*96.0;
    y2 = -96.0*tail.y + 0.5*11.0*96.0;

    svg_file += "\n<line ";
    svg_file += "x1=\"" + std::to_string(x1) + "\"";
    svg_file += " y1=\"" + std::to_string(y1) + "\"";
    svg_file += " x2=\"" + std::to_string(x2) + "\"";
    svg_file += " y2=\"" + std::to_string(y2) + "\"";
    svg_file += " stroke=\"" + color + "\" stroke-width=\"5.0\" marker-start=\"url(#Arrow1Sstart)\"/>";

}

void turtlelib::BuildSVG::Draw(turtlelib::Transform2D T, std::string label)
{
    double theta = T.rotation(); // const auto
    double x0 = T.translation().x;
    double y0 = T.translation().y;
    double x1 = x0 + cos(theta);
    double y1 = y0 + sin(theta);
    double x2 = x0 - sin(theta);
    double y2 = y0 + cos(theta);

    svg_file += "\n<g>";
    turtlelib::Point2D head;
    head.x = x1;
    head.y = y1;
    turtlelib::Point2D tail;
    tail.x = x0;
    tail.y = y0;
    Draw(head, tail, "blue");
    turtlelib::Point2D head2;
    head2.x = x2;
    head2.y = y2;
    Draw(head2, tail, "red");
    x0 = 96.0*x0 + 0.5*8.5*96.0;
    y0 = -96.0*y0 + 0.5*11.0*96.0;
    svg_file += "<text";
    svg_file += " x=\"" + std::to_string(x0) + "\"";
    svg_file += " y=\"" + std::to_string(y0) + "\"";
    svg_file += " font-size=\"12.0\" fill=\"black\">" + label + "</text>";
    
    svg_file += "\n</g>";

}

