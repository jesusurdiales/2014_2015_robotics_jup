/**
 * @file    MyRobot.h
 * @brief   Program that allows the robot to follow the walls.
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

#define MAX_SPEED       50
#define NUMBER_OF_DSENSORS 4
#define MAX_DISTANCE 100

class MyRobot : public DifferentialWheels
{
    private:
        int _time_step;

        // Declaration of all distance sensors
        DistanceSensor * _dsensors[NUMBER_OF_DSENSORS];

        // Declaration of all movement modes
        enum Mode
        {
            STOP, TURN_RIGHT, TURN_LEFT, FORWARD, BACK_RIGHT, BACK_LEFT
        };
        Mode _mov_mode;

        // This state difines wheter the robot is following a wall or not.
        enum State
        {
            FREE, FOLLOWING_RIGHT_WALL, FOLLOWING_LEFT_WALL
        };
        State _robot_state;

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
         * @brief Funtion that implements robot behavior: when it's close to walls, the robot follow them.
         */
        void run();

        /**
         * @brief Function that controls free (initial) move. The robot goes straight until it finds a wall.
         * @param ds1_dist - distance measured by sensor 1.
         * @param ds14_dist - distance measured by sensor 14.
         */
        void move_free (double ds1_dist, double ds14_dist);

        /**
         * @brief Function that controls the robot while it is following the right wall.
         * @param ds1_dist - distance measured by sensor 1.
         * @param ds13_dist - distance measured by sensor 13.
         * @param ds14_dist - distance measured by sensor 14.
         */
        void following_right_wall_state (double ds1_dist, double ds13_dist, double ds14_dist);

        /**
         * @brief Function that controls the robot while it is following the right wall.
         * @param ds1_dist - distance measured by sensor 1.
         * @param ds2_dist - distance measured by sensor 2.
         * @param ds14_dist - distance measured by sensor 14.
         */
        void following_left_wall_state (double ds1_dist, double ds2_dist, double ds14_dist);

        /**
         * @brief Function used for set robot mode of move.
         */
        void do_move ();

        /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          */
        double convert_bearing_to_degrees(const double* in_vector);
};
