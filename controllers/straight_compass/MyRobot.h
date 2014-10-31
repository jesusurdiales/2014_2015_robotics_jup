/**
 * @file    MyRobot.h
 * @brief   A simple program for maintaining a straight line with the compass.
 *
 * @author  Jesús Urdiales de la Parra
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100
// This is the angle the robot will maintain
#define DESIRED_ANGLE   45

class MyRobot : public DifferentialWheels
{
    private:
        int _time_step;
        // Declaration of the compass device
        Compass * _my_compass;
        double _left_speed, _right_speed;

    public:
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief Function that contains all the necessary logic needed for robot movement
         */
        void run();

        /**
          * @brief Function that implements compass_movement
          * @param compass_angle Direction returned by the compass in degrees
          */
        void compass_move(double compass_angle);

        /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          */
        double convert_bearing_to_degrees(const double* in_vector);
};
