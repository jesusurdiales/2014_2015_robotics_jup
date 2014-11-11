/**
 * @file    MyRobot.cpp
 * @brief   Program that shows the robot using odometry to move across the map.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    // Both encoders of wheels are enabled whith this instruction
    enableEncoders(_time_step);

    _left_speed = 0;
    _right_speed = 0;
    _left_encoder = 0;
    _right_encoder = 0;

    _odometry_error = 0.0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // In this program the robot only have to go straight for 17.5 meters
    double desired_distance = 17.5;
    double actual_distance = 0.0;

    // Calculate how many encoders ticks are equivalents to 0.5 cm of error:
    // 1rad -> ENCODER_RESOLUTION ticks -> WHEEL_RADIUS m of error
    _odometry_error = 0.005 * ENCODER_RESOLUTION / WHEEL_RADIUS;

    while (step(_time_step) != -1)
    {
        // Read the enconders
        _left_encoder = getLeftEncoder();
        _right_encoder = getRightEncoder();

        // Caculate the distance with odometry
        actual_distance = _left_encoder / ENCODER_RESOLUTION * WHEEL_RADIUS;

        // Show values for distance and encoders
        cout << "Left encoder: " << _left_encoder << " Right encoder: " << _right_encoder << " Distance: " << actual_distance
                << endl;

        // If the robot runs for more than _desired_distance meters, it stops
        if (actual_distance < desired_distance)
            do_straight_move();
        else
        {
            setSpeed (0,0);
            cout << "I cross the map, finishing program"  << endl;
            break;
        }
    }
}

//////////////////////////////////////////////

void MyRobot::do_straight_move ()
{
    // Calculate wheels speed according to odometry error
    if ((_left_encoder - _right_encoder) > _odometry_error)
    {
        // The robot must to turn left
        _left_speed = MAX_SPEED / 1.25;
        _right_speed = MAX_SPEED;
        setSpeed (_left_speed, _right_speed);
    }
    else
    {
        // The robot must to turn right
        if ((_right_encoder - _left_encoder) > _odometry_error)
        {
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED / 1.25;
            setSpeed (_left_speed, _right_speed);
        }
        // The robot can go straight
        else
            setSpeed (MAX_SPEED, MAX_SPEED);
    }
}
