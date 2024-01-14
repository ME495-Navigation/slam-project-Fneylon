#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

// using namepace turtlelib;
// using namespace Transfrom2D;


TEST_CASE("Testing operator<<", "[operator_se2<<]")
{
    turtlelib::Twist2D tw;
    tw.omega = 5.0;
    tw.x = 6.0;
    tw.y = 7.0;
    std::stringstream ss1;
    ss1 << tw;
    REQUIRE(ss1.str() == "[5 6 7]");

    turtlelib::Vector2D v;
    v.x = 3.0;
    v.y = 4.0;
    double theta = turtlelib::deg2rad(90.0);
    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);
    std::stringstream ss2;
    ss2 << T;
    REQUIRE(ss2.str() == "deg: 90 x: 3 y: 4");

}

TEST_CASE("Testing operator_se2>>", "[operator_se2>>]")
{
    turtlelib::Twist2D tw;
    std::stringstream ss1;
    ss1 << "[5 6 7]";
    ss1 >> tw;
    REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(6.0, 1.0e-12));
    REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(7.0, 1.0e-12));

    turtlelib::Twist2D tw2;
    std::stringstream ss2;
    ss2 << "5 6 7";
    ss2 >> tw2;
    REQUIRE_THAT(tw2.omega, Catch::Matchers::WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(tw2.x, Catch::Matchers::WithinAbs(6.0, 1.0e-12));
    REQUIRE_THAT(tw2.y, Catch::Matchers::WithinAbs(7.0, 1.0e-12));

    std::stringstream ss3;
    turtlelib::Transform2D T;
    // double theta = 90.0;
    ss3 << "deg: 90 x: 3 y: 4";
    ss3 >> T;
    // REQUIRE_THAT(turtlelib::rad2deg(T.rotatcd ..ion()), Catch::Matchers::WithinAbs(theta, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(4.0, 1.0e-12));


}

TEST_CASE("Testing Transform2D()", "[Transform2D]")
{
    turtlelib::Transform2D t;

    REQUIRE_THAT(t.matrix[0][0], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[0][1], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[0][2], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[1][0], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[1][1], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[1][2], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[2][0], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[2][1], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(t.matrix[2][2], Catch::Matchers::WithinAbs(1.0, 1.0e-12));

    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    turtlelib::Transform2D T = turtlelib::Transform2D(v);

    REQUIRE_THAT(T.matrix[0][2], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(T.matrix[1][2], Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    double theta = turtlelib::deg2rad(90.0);
    turtlelib::Transform2D T2 = turtlelib::Transform2D(theta);

    REQUIRE_THAT(T2.matrix[0][0], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T2.matrix[0][1], Catch::Matchers::WithinAbs(-1.0, 1.0e-12));
    REQUIRE_THAT(T2.matrix[1][0], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(T2.matrix[1][1], Catch::Matchers::WithinAbs(0.0, 1.0e-12));


    turtlelib::Transform2D T3 = turtlelib::Transform2D(v, theta);
    REQUIRE_THAT(T3.matrix[0][0], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T3.matrix[0][1], Catch::Matchers::WithinAbs(-1.0, 1.0e-12));
    REQUIRE_THAT(T3.matrix[1][0], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(T3.matrix[1][1], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T3.matrix[0][2], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(T3.matrix[1][2], Catch::Matchers::WithinAbs(2.0, 1.0e-12));
}

TEST_CASE("Testing Operator", "[operator]")
{
    turtlelib::Point2D p1;
    p1.x = 1.0;
    p1.y = 2.0;
    
    double theta = turtlelib::deg2rad(90.0);
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);
    turtlelib::Point2D p2 = T(p1);


    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(-1.0, 1.0e-12));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(3.0, 1.0e-12));

    turtlelib::Vector2D v2;
    v2.x = 1.0;
    v2.y = 2.0;

    turtlelib::Vector2D v3 = T(v2);
    REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(-1.0, 1.0e-12));
    REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(3.0, 1.0e-12));


    turtlelib::Twist2D tw;
    tw.omega = 5.0;
    tw.x = 6.0;
    tw.y = 7.0;

    turtlelib::Twist2D tw2 = T(tw);
    REQUIRE_THAT(tw2.omega, Catch::Matchers::WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(tw2.x, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
    REQUIRE_THAT(tw2.y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));


}

TEST_CASE("Testing inv", "[inv]"){

    turtlelib::Vector2D v;
    v.x = 0.0;
    v.y = 0.0;

    turtlelib::Transform2D T = turtlelib::Transform2D(v, 0.0);

    turtlelib::Transform2D Tinv = T.inv();

    REQUIRE_THAT(Tinv.matrix[0][0], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[0][1], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[0][2], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[1][0], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[1][1], Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[1][2], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[2][0], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[2][1], Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(Tinv.matrix[2][2], Catch::Matchers::WithinAbs(1.0, 1.0e-12));

}

TEST_CASE("Testing translation", "[translation]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    turtlelib::Transform2D T = turtlelib::Transform2D(v);

    turtlelib::Vector2D v2 = T.translation();

    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));


}

TEST_CASE("Testing rotation","[rotation]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    double gamma = turtlelib::rad2deg(T.rotation());

    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(90.0, 1.0e-12));

}