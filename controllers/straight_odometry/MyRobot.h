/**
 * @file    MyRobot.h
 * @brief   Program that shows the robot using odometry to move across the map.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace webots;
using namespace std;

#define MAX_SPEED           50
#define ENCODER_RESOLUTION  5
#define WHEEL_RADIUS        0.0825

class MyRobot : public DifferentialWheels
{
private:

    // Variables to store encoder values
    double _left_encoder, _right_encoder;
    // Variables to control wheels speed
    double _left_speed, _right_speed;

    // Robot error permited
    double _odometry_error;

    int _time_step;

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
     * @brief Function that reads encoders values and calculates the distance the robot has move.
     */
    void run();

    /**
     * @brief Function that calculates wheel speed necesary to maintain the robot straight.
     */
    void do_straight_move ();
};
