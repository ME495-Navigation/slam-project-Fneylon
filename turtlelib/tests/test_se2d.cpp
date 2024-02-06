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
    turtlelib::Twist2D tw; // constructor
    tw.omega = 5.0;
    tw.x = 6.0;
    tw.y = 7.0;
    std::stringstream ss1;
    ss1 << tw;
    REQUIRE(ss1.str() == "[5 6 7]");

    turtlelib::Vector2D v; // constructor
    v.x = 3.0;
    v.y = 4.0;
    double theta = turtlelib::deg2rad(90.0); // const auto
    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);
    std::stringstream ss2;
    ss2 << T;
    REQUIRE(ss2.str() == "deg: 90 x: 3 y: 4");

}

TEST_CASE("Testing operator>>", "[operator_se2>>]")
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
    ss3 << "90 3 4";
    ss3 >> T;
    double x = T.translation().x;
    double y = T.translation().y;
    double gamma = turtlelib::rad2deg(T.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(90.0, 1.0e-12));
    REQUIRE_THAT(x, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
    REQUIRE_THAT(y, Catch::Matchers::WithinAbs(4.0, 1.0e-12));


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
    REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));


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
    double gamma = turtlelib::deg2rad(90.0);
    v.x = 1.0;
    v.y = 2.0;

    turtlelib::Transform2D T = turtlelib::Transform2D(v, gamma);

    turtlelib::Transform2D Tinv = T.inv();
    gamma = turtlelib::rad2deg(Tinv.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(-90.0, 1.0e-12));
    REQUIRE_THAT(Tinv.translation().x, Catch::Matchers::WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(Tinv.translation().y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));

}

TEST_CASE("Testing translation", "[translation]")
{
    turtlelib::Vector2D v; // constructor
    v.x = 1.0;
    v.y = 2.0;

    turtlelib::Transform2D T = turtlelib::Transform2D(v);

    turtlelib::Vector2D v2 = T.translation();

    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));


}

TEST_CASE("Testing rotation","[rotation]")
{
    turtlelib::Vector2D v; // constructor
    v.x = 1.0;
    v.y = 2.0;

    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    double gamma = turtlelib::rad2deg(T.rotation());

    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(90.0, 1.0e-12));

}

TEST_CASE("Testing operator*=", "[operator*=]")
{
    turtlelib::Vector2D v; // constructor
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    turtlelib::Vector2D v2; // constructor
    v2.x = 3.0;
    v2.y = 4.0;
    double theta2 = turtlelib::deg2rad(180.0);
    turtlelib::Transform2D T1 = turtlelib::Transform2D(v2, theta2);

    T*=T1;

    double gamma = turtlelib::rad2deg(T.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(270.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-3.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));


}

TEST_CASE("Testing operator*", "[operator*]")
{
    turtlelib::Vector2D v; // use the constructor
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;
    double theta2 = turtlelib::deg2rad(180.0);
    turtlelib::Transform2D T1 = turtlelib::Transform2D(v2, theta2);

    turtlelib::Transform2D T2 = T*T1;

    double gamma = turtlelib::rad2deg(T2.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(270.0, 1.0e-12));
    REQUIRE_THAT(T2.translation().x, Catch::Matchers::WithinAbs(-3.0, 1.0e-12));
    REQUIRE_THAT(T2.translation().y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));

}
