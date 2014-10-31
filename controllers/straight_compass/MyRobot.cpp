/**
 * @file    MyRobot.cpp
 * @brief   A simple program for maintaining a straight line with the compass.
 *
 * @author  Jesús Urdiales de la Parra
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // We have to disable all devices in exit
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;

    while (step(_time_step) != -1)
    {
        // Read the sensors
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

        // If the simulation runs for more than 23 seconds, the robot will stop to avoid the wall
        if (getTime() > 23)
        {
            setSpeed(0,0);
            break;
        }
        // Move the robot according compass angle
        compass_move(compass_angle);
    }
}

void MyRobot::compass_move(double compass_angle)
{
    // Simple bang-bang control
    if (compass_angle < (DESIRED_ANGLE - 2)) {
        // Turn right
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED - 15;
    }
    else {
        if (compass_angle > (DESIRED_ANGLE + 2)) {
            // Turn left
            _left_speed = MAX_SPEED - 15;
            _right_speed = MAX_SPEED;
        }
        else {
            // Move straight forward
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
        }
    }

    // Set the motor speeds
    setSpeed(_left_speed, _right_speed);
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
