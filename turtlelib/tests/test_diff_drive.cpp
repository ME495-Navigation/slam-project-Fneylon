#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

TEST_CASE("Testing drive_forward", "[drive_forward]"){

    turtlelib::DiffDrive diff_drive;
    double wheel_radius = 0.5;
    double wheel_track = 1.0;
    diff_drive = turtlelib::DiffDrive(wheel_radius, wheel_track);

    // We need the robot to move forward
    turtlelib::WheelConfiguration w;
    w.theta_l = turtlelib::PI/4.0;
    w.theta_r = turtlelib::PI/4.0;

    diff_drive.forward_kinematics(w);
    turtlelib::Configuration2D c;
    c = diff_drive.get_configuration(); // Pose x, y, theta

    std::cout <<"X After first roll: "<< c.x << std::endl;
    std::cout <<"Y After first roll: "<< c.y << std::endl;
    std::cout <<"theta After first roll: "<< c.theta << std::endl;

    REQUIRE_THAT(c.x, Catch::Matchers::WithinAbs(0.5 *turtlelib::PI/4, 1.0e-12));
    REQUIRE_THAT(c.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    // Move the robot forward again
    w.theta_l = turtlelib::PI/2.0;
    w.theta_r = turtlelib::PI/2.0;

    diff_drive.forward_kinematics(w);
    c = diff_drive.get_configuration(); // Pose x, y, theta

    std::cout <<"X After second roll: "<< c.x << std::endl;
    std::cout <<"Y After second roll: "<< c.y << std::endl;
    std::cout <<"theta After second roll: "<< c.theta << std::endl;

    REQUIRE_THAT(c.x, Catch::Matchers::WithinAbs(turtlelib::PI/4, 1.0e-12));
    REQUIRE_THAT(c.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    // Move the robot forward again
    w.theta_l = turtlelib::PI*3/4.0;
    w.theta_r = turtlelib::PI*3/4.0;

    diff_drive.forward_kinematics(w);
    c = diff_drive.get_configuration(); // Pose x, y, theta

    std::cout <<"X After third roll: "<< c.x << std::endl;
    std::cout <<"Y After third roll: "<< c.y << std::endl;
    std::cout <<"theta After third roll: "<< c.theta << std::endl;

    REQUIRE_THAT(c.x, Catch::Matchers::WithinAbs(3*turtlelib::PI/8, 1.0e-12));
    REQUIRE_THAT(c.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    // Move the robot backwards 
    w.theta_l = turtlelib::PI/2.0;
    w.theta_r = turtlelib::PI/2.0;

    diff_drive.forward_kinematics(w);
    c = diff_drive.get_configuration(); // Pose x, y, theta

    std::cout <<"X After fourth roll: "<< c.x << std::endl;
    std::cout <<"Y After fourth roll: "<< c.y << std::endl;
    std::cout <<"theta After fourth roll: "<< c.theta << std::endl;

    REQUIRE_THAT(c.x, Catch::Matchers::WithinAbs(turtlelib::PI/4, 1.0e-12));
    REQUIRE_THAT(c.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    // Have the robot do nothing
    w.theta_l = turtlelib::PI/2.0;
    w.theta_r = turtlelib::PI/2.0;

    diff_drive.forward_kinematics(w);
    c = diff_drive.get_configuration(); // Pose x, y, theta

    std::cout <<"X After fifth roll: "<< c.x << std::endl;
    std::cout <<"Y After fifth roll: "<< c.y << std::endl;
    std::cout <<"theta After fifth roll: "<< c.theta << std::endl;

    REQUIRE_THAT(c.x, Catch::Matchers::WithinAbs(turtlelib::PI/4, 1.0e-12));
    REQUIRE_THAT(c.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    // REQUIRE(0 ==1);

}



TEST_CASE("Testing forward_kinematics", "[forward_kinematics]")
{

// Forward Kinmatics:
    turtlelib::DiffDrive dd;
    double wheel_radius = 0.5;
    double wheel_track = 1.0;
    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);


    // Robot Does Nothing
    turtlelib::WheelConfiguration w1;
    w1.theta_l = 0.0;
    w1.theta_r = 0.0;

    turtlelib::Configuration2D c1;
    dd.forward_kinematics(w1);
    c1 = dd.get_configuration();
    REQUIRE_THAT(c1.x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c1.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c1.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    turtlelib::Twist2D twist;
    twist.omega = 0.0;
    twist.x = 0.0;
    twist.y = 0.0;
    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::WheelConfiguration w;
    w = dd.inverse_kinematics(twist);
    // dd.inverse_kinematics(T);
    REQUIRE_THAT(w.theta_l, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(w.theta_r, Catch::Matchers::WithinAbs(0.0, 1.0e-12));


    // Robot Moves Forward
    turtlelib::WheelConfiguration w2;
    w2.theta_l = turtlelib::PI/2.0;
    w2.theta_r = turtlelib::PI/2.0;

    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::Configuration2D c2;
    dd.forward_kinematics(w2);
    c2 = dd.get_configuration();


    REQUIRE_THAT(c2.x, Catch::Matchers::WithinAbs(0.5 *turtlelib::PI/2, 1.0e-12));
    REQUIRE_THAT(c2.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c2.theta, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    turtlelib::Twist2D twist2;
    twist2.omega = 0.0;
    twist2.x = 0.5 *turtlelib::PI/2.0;
    twist2.y = 0.0;
    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::WheelConfiguration w_test;
    w_test = dd.inverse_kinematics(twist2);
    REQUIRE_THAT(w_test.theta_l, Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-12));
    REQUIRE_THAT(w_test.theta_r, Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-12));

    // Robot Executes Pure Rotation
    turtlelib::WheelConfiguration w3;
    w3.theta_l = turtlelib::PI/2.0;
    w3.theta_r = -turtlelib::PI/2.0;

    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::Configuration2D c3;
    dd.forward_kinematics(w3);
    c3 = dd.get_configuration();

    REQUIRE_THAT(c3.x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c3.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(c3.theta, Catch::Matchers::WithinAbs(-turtlelib::PI/2, 1.0e-12));

    turtlelib::Twist2D twist3;
    twist3.omega = -turtlelib::PI/2.0;
    twist3.x = 0.0;
    twist3.y = 0.0;
    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::WheelConfiguration w_test3;
    w_test3 = dd.inverse_kinematics(twist3);
    REQUIRE_THAT(w_test3.theta_l, Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-12));
    REQUIRE_THAT(w_test3.theta_r, Catch::Matchers::WithinAbs(-turtlelib::PI/2, 1.0e-12));

    // Robot Execute an Arc
    turtlelib::WheelConfiguration w4;
    w4.theta_l = turtlelib::PI/2.0;
    w4.theta_r = turtlelib::PI/4.0;

    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::Configuration2D c4;
    dd.forward_kinematics(w4);
    c4 = dd.get_configuration();

    turtlelib::Configuration2D c_test;
    c_test.x = -3.0/2.0 * sin(-turtlelib::PI/8.0);
    c_test.y = -3.0/2.0 + 3.0/2.0 * cos(-turtlelib::PI/8.0);
    c_test.theta = -turtlelib::PI/8.0;

    REQUIRE_THAT(c4.x, Catch::Matchers::WithinAbs(c_test.x, 1.0e-12));
    REQUIRE_THAT(c4.y, Catch::Matchers::WithinAbs(c_test.y, 1.0e-12));
    REQUIRE_THAT(c4.theta, Catch::Matchers::WithinAbs(c_test.theta, 1.0e-12));
    
    dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    turtlelib::Twist2D twist4;
    twist4.omega = -turtlelib::PI/8.0;
    twist4.x = 3.0*turtlelib::PI/16.0;
    twist4.y = 0.0;
    turtlelib::WheelConfiguration w_test4;
    w_test4 = dd.inverse_kinematics(twist4);
    REQUIRE_THAT(w_test4.theta_l, Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-12));
    REQUIRE_THAT(w_test4.theta_r, Catch::Matchers::WithinAbs(turtlelib::PI/4, 1.0e-12));

    // dd = turtlelib::DiffDrive(wheel_radius, wheel_track);
    // turtlelib::Twist2D twist4;
    // twist4.omega = -turtlelib::PI/8.0;
    // twist4.x = 3.0*turtlelib::PI/16.0;
    // twist4.y = 1.0;
    // turtlelib::WheelConfiguration w_test4;    

}