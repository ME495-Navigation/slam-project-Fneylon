// #define CATCH_CONFIG_MAIN
#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

// Edit to use REQUIRE_THAT to use the MATCHER 

// Test Case for almost_equal
TEST_CASE("Test almost_equal", "[almost_equal]")
{
    REQUIRE(turtlelib::almost_equal(0.0, 0.0) == true);
    REQUIRE(turtlelib::almost_equal(0.1, 1.0) == false);
    REQUIRE(turtlelib::almost_equal(4.500000000000001, 4.500000000000002) == true);
}

// Test Case for deg2rad
TEST_CASE("Test deg2rad", "[deg2rad]")
{
    REQUIRE_THAT(turtlelib::deg2rad(0.0), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::deg2rad(180.0), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-12));
    REQUIRE_THAT(turtlelib::deg2rad(360.0), Catch::Matchers::WithinAbs( 2 * turtlelib::PI, 1.0e-12));

}

TEST_CASE("Test rad2deg", "[rad2deg]")
{
    REQUIRE_THAT(turtlelib::rad2deg(0.0), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::rad2deg(turtlelib::PI), Catch::Matchers::WithinAbs(180.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::rad2deg(2 * turtlelib::PI), Catch::Matchers::WithinAbs(360.0, 1.0e-12));
}

TEST_CASE("Testing normalize_angle", "[normalize_angle]")
{
    REQUIRE_THAT(turtlelib::normalize_angle(turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI / 4), Catch::Matchers::WithinAbs(-turtlelib::PI / 4, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(3 * turtlelib::PI / 2), Catch::Matchers::WithinAbs(-turtlelib::PI / 2, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI*5/2), Catch::Matchers::WithinAbs(-turtlelib::PI / 2, 1e-12));

}

TEST_CASE("Testing operator<<", "[operator<<]")
{
    turtlelib::Point2D p;
    p.x = 1.0;
    p.y = 2.0;
    std::stringstream ss1;
    ss1 << p;
    REQUIRE(ss1.str() == "[1 2]");

    turtlelib::Vector2D v;
    v.x = 3.0;
    v.y = 4.0;
    std::stringstream ss2;
    ss2 << v;
    REQUIRE(ss2.str() == "[3 4]");

}

TEST_CASE("Testing operator>>", "[operator>>]")
{
    turtlelib::Point2D p;
    std::stringstream ss1;
    ss1 << "[1 2]";
    ss1 >> p;
    REQUIRE(p.x == 1.0);
    REQUIRE(p.y == 2.0);

    std::stringstream ss2;
    ss2 << "1 3";
    ss2 >> p;
    REQUIRE(p.x == 1.0);
    REQUIRE(p.y == 3.0);


    turtlelib::Vector2D v;
    std::stringstream ss3;
    ss3 << "[3 4]";
    ss3 >> v;
    REQUIRE(v.x == 3.0);
    REQUIRE(v.y == 4.0);

    std::stringstream ss4;
    ss4 << "3 5";
    ss4 >> v;
    REQUIRE(v.x == 3.0);
    REQUIRE(v.y == 5.0);

}

TEST_CASE("Testing operator-", "[operator-]")
{
    turtlelib::Point2D p1;
    p1.x = 1.0;
    p1.y = 2.0;
    turtlelib::Point2D p2;
    p2.x = 3.0;
    p2.y = 4.0;
    turtlelib::Vector2D v = p1 - p2;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(-2.0, 1e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(-2.0, 1e-12));

    turtlelib::Vector2D v1;
    v1.x = 1.0;
    v1.y = 2.0;
    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;

    v1 -=v2;
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(-2.0, 1e-12));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(-2.0, 1e-12));

}

TEST_CASE("Testing operator+", "[operator+]")
{
    turtlelib::Point2D p1;
    p1.x = 1.0;
    p1.y = 2.0;
    turtlelib::Vector2D v;
    v.x = 3.0;
    v.y = 4.0;
    turtlelib::Point2D p2 = p1 + v;
    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(6.0, 1e-12));

    turtlelib::Vector2D v1;
    v1.x = 1.0;
    v1.y = 2.0;
    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;

    v1 +=v2;

    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(6.0, 1e-12));

}

TEST_CASE("Testing vec_operator*", "[vec_operator*]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    turtlelib::Vector2D v1 = v * 3.0;
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(3.0, 1e-12));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(6.0, 1e-12));

    turtlelib::Vector2D v2;
    v2.x = 1.0;
    v2.y = 2.0;
    v2 *= 3.0;
    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(3.0, 1e-12));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(6.0, 1e-12));

}

TEST_CASE("Testing dot", "[dot]")
{
    turtlelib::Vector2D v1;
    v1.x = 1.0;
    v1.y = 2.0;
    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;
    double dot = turtlelib::dot(v1, v2);
    REQUIRE_THAT(dot, Catch::Matchers::WithinAbs(11.0, 1e-12));

}

TEST_CASE("Testing magnitude", "[magnitude]")
{
    turtlelib::Vector2D v;
    v.x = 3.0;
    v.y = 4.0;
    double mag = turtlelib::magnitude(v);
    REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(5.0, 1e-12));

}

TEST_CASE("Testing angle", "[angle]")
{
    turtlelib::Vector2D v1;
    v1.x = 1.0;
    v1.y = 0.0;
    turtlelib::Vector2D v2;
    v2.x = 0.0;
    v2.y = 1.0;
    double angle = turtlelib::angle(v1, v2);
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(turtlelib::PI / 2, 1e-12));

}

// TEST_CASE("Testing normalize", "[normalize]")
// {
//     turtlelib::Vector2D v;
//     v.x = 3.0;
//     v.y = 4.0;
//     turtlelib::Vector2D v2;
//     v2 = turtlelib::normalize(v);
//     REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(0.6, 1e-12));
//     REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(0.8, 1e-12));
// }