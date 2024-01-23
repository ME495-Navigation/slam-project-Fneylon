#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

TEST_CASE ("Testing ExportingSVG", "[ExportingSVG]")
{


    turtlelib::BuildSVG svg;
    turtlelib::Point2D point;
    point.x = 0.0;
    point.y = 0.0;
    std::string color = "black";
    svg.Draw(point, color);

    turtlelib::Transform2D T;
    turtlelib::Vector2D v;
    v.x = 3.0;
    v.y = 4.0;
    double theta = turtlelib::deg2rad(45.0);
    T = turtlelib::Transform2D(v, theta);
    std::string label = "{a}";
    svg.Draw(T, label);

    std::string svg_string;
    svg_string = "\n<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    svg_string += "\n<defs>";
    svg_string += "\n\t<marker";
    svg_string += "\n\t\tstyle=\"overflow:visible\"";
    svg_string += "\n\t\tid=\"Arrow1Sstart\"";
    svg_string += "\n\t\trefX=\"0.0\"";
    svg_string += "\n\t\trefY=\"0.0\"";
    svg_string += "\n\t\torient=\"auto\">";
    svg_string += "\n\t\t\t<path";
    svg_string += "\n\t\t\t\ttransform=\"scale(0.2) translate(6,0)\"";
    svg_string += "\n\t\t\t\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"";
    svg_string += "\n\t\t\t\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"";
    svg_string += "\n\t\t\t/>";
    svg_string += "\n\t</marker>";
    svg_string += "\n</defs>";

    // Defining the Black Circle
    double cx = 96.0*point.x + 0.5*8.5*96.0;
    double cy = -96.0*point.y + 0.5*11.0*96.0;
    svg_string += "\n<circle ";
    svg_string += "cx=\"" + std::to_string(cx) + "\"";
    svg_string += " cy=\"" + std::to_string(cy) + "\"";
    svg_string += " r=\"3.0\"";
    svg_string += " fill=\"" + color + "\" stroke=\"" + color + "\" stroke-width=\"1.0\"/>";

    svg_string += "\n<g>";
    svg_string += "\n<line x1=\"763.882251\" y1=\"76.117749\" x2=\"696.000000\" y2=\"144.000000\" stroke=\"blue\" stroke-width=\"5.0\" marker-start=\"url(#Arrow1Sstart)\"/>";
    svg_string += "\n<line x1=\"628.117749\" y1=\"76.117749\" x2=\"696.000000\" y2=\"144.000000\" stroke=\"red\" stroke-width=\"5.0\" marker-start=\"url(#Arrow1Sstart)\"/><text x=\"696.000000\" y=\"144.000000\" font-size=\"12.0\" fill=\"black\">{a}</text>";
    svg_string += "\n</g>";
    svg_string += "\n</svg>";
    REQUIRE(svg.ExportSVG() == svg_string);
}
