/**
 * @file    MyRobot.h
 * @brief   A program that shows how the robot avoids obstacles in an unknown enviroment.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUMBER_OF_DSENSORS  16
#define MAX_DISTANCE        100
#define MAX_SPEED           30
// This is the angle of the target direction
#define DESIRED_ANGLE       45

class MyRobot : public DifferentialWheels
{
private:
    int _time_step;
    // Declaration of the compass
    Compass *_objetive_compass;

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
        FOLLOWING_OBJETIVE, FOLLOWING_RIGHT_WALL, FOLLOWING_LEFT_WALL
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
         * @brief Function that contains the logic of robot movement in obstacle avoidance.
         */
        void run();
        /**
         * @brief Function to control robot's movement with the compass. When there are no obstacles, the robot follows compass
         * direction to reach the goal.
         * @param ds_values Array of doubles that contains the distance sensors values.
         * @param compass_angle - angle returned by the compass, in degrees.
         */
        void following_objetive_state (double ds_values[], double compass_angle);

        /**
         * @brief Function that controls the robot while it is following the right wall.
         * @param ds_values Array of doubles that contains the distance sensors values.
         */
        void following_right_wall_state (double ds_values[]);

        /**
         * @brief Function that controls the robot while it is following the left wall.
         * @param ds_values Array of doubles that contains the distance sensors values.
         */
        void following_left_wall_state (double ds_values[]);

        /**
         * @brief Function used for set robot mode of move.
         */
        void do_move ();

        /**
         * @brief Function that reads distance sensors values.
         * @param ds_values Array of doubles that contains old distance sensors values.
         */
        void read_distance_sensors(double *ds_values);

        /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          * @param in_vector Vector of values returned by the compass.
          */
        double convert_bearing_to_degrees(const double* in_vector);
};
