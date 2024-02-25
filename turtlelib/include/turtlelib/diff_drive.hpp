#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
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
    /// \brief A 2D configuration
    struct Configuration2D
    {
        /// \brief the x position
        double x = 0.0;

        /// \brief the y position
        double y = 0.0;

        /// \brief the orientation
        double theta = 0.0;
    };

    /// \brief Wheel configuration for 2 wheel robot
    struct WheelConfiguration
    {
        /// \brief left wheel configuration
        double theta_l = 0.0;

        /// \brief right wheel configuration
        double theta_r = 0.0;
    };


    /// \brief A differential drive model class 
    class DiffDrive
    {
        public:

            /// \brief Create an empty differential drive class
            DiffDrive();

            /// \brief create a differential drive model
            /// \param wheel_radius - the radius of the wheels
            /// \param wheel_track - the distance between the wheels
            DiffDrive(double wheel_radius, double wheel_track);

            /// \brief compute the forward kinematics of the robot
            /// \param wheels - the wheel configuration
            void forward_kinematics(WheelConfiguration wheels);

            /// \brief compute the inverse kinematics of the robot
            /// \param twist - the twist to apply
            /// \return the wheel configuration
            WheelConfiguration inverse_kinematics(Twist2D twist);

            /// \brief get the current configuration of the robot
            /// \return the current configuration
            Configuration2D get_configuration();

            /// \brief get the current twist of the robot
            /// \return the current twist
            Twist2D get_Twist();

            /// \brief set the current configuration of the robot
            /// \param x - the x position
            /// \param y - the y position
            /// \param theta - the orientation
            void set_configuration(double x, double y, double theta);

        private:

            double wheel_radius_;
            double wheel_track_;

            WheelConfiguration wheel_config_;
            Configuration2D config_;

            Twist2D twist_;
    };
}

#endif // TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP