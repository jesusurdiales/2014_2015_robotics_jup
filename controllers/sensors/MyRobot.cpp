/**
 * @file    MyRobot.cpp
 * @brief   A simple program for showing distance sensors values while the robot is moving.
 *
 * @author  Jesús Urdiales de la Parra
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////
// Constructor of the class robot
MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    
    // Get the distance sensors
    _dsensors[0] = getDistanceSensor("ds0");
    _dsensors[1] = getDistanceSensor("ds1");
    _dsensors[2] = getDistanceSensor("ds2");
    _dsensors[3] = getDistanceSensor("ds3");
    _dsensors[4] = getDistanceSensor("ds12");
    _dsensors[5] = getDistanceSensor("ds13");
    _dsensors[6] = getDistanceSensor("ds14");
    _dsensors[7] = getDistanceSensor("ds15");

    // Enable all distance sensors
    for (int i = 0; i < NUMBER_OF_DSENSORS; i++)
    {
        _dsensors[i]->enable(_time_step);
    }
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Disable all distance sensors and the compass
    _my_compass->disable();

    for (int i = 0; i < NUMBER_OF_DSENSORS; i++)
    {
        _dsensors[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{ 
    double compass_angle;

    while (step(_time_step) != -1) {
    
        //Leemos los sensores de distancia e imprimimos su valor por pantalla
        cout << "Sensor 0: " << _dsensors[0]->getValue() << endl;
        cout << "Sensor 1: " << _dsensors[1]->getValue() << endl;
        cout << "Sensor 2: " << _dsensors[2]->getValue() << endl;
        cout << "Sensor 3: " << _dsensors[3]->getValue() << endl;
        cout << "Sensor 12: " << _dsensors[4]->getValue() << endl;
        cout << "Sensor 13: " << _dsensors[5]->getValue() << endl;
        cout << "Sensor 14: " << _dsensors[6]->getValue() << endl;
        cout << "Sensor 15: " << _dsensors[7]->getValue() << endl;
        
        // Read the compass
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print compass values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

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
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
