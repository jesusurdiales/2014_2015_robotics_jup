/**
 * @file    MyRobot.h
 * @brief   A simple program for showing distance sensors values while the robot is moving.
 *
 * @author  Jesús Urdiales de la Parra
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100
// This is the angle the robot will maintain
#define DESIRED_ANGLE   45
#define NUMBER_OF_DSENSORS 8

class MyRobot : public DifferentialWheels
{
    private:
        int _time_step;
        // Declaration of the compass
        Compass * _my_compass;
        // Declaration of all distance sensors
        DistanceSensor * _dsensors[NUMBER_OF_DSENSORS];

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
         * @brief Function that contains all the necessary logic needed for robot behavior.
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
