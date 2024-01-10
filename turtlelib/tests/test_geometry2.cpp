#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>


// Test Case for almost_equal
TEST_CASE("Test almost_equal", "[almost_equal]")
{

    //Test Case 1
    REQUIRE(turtlelib::almost_equal(0.0, 0.0) == true);
    REQUIRE(turtlelib::almost_equal(0.0, 0, 0.1) == false);
    REQUIRE(turtlelib::almost_equal(4.500000000000001, 4.500000000000002) == true);
}
