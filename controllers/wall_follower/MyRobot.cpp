/**
 * @file    MyRobot.cpp
 * @brief   Program that allows the robot to follow the walls.
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

    // Initial movement and state mode
    _mov_mode = FORWARD;
    _robot_state = FREE;
    
    // Get the distance sensors
    _dsensors[0] = getDistanceSensor("ds1");
    _dsensors[1] = getDistanceSensor("ds2");
    _dsensors[2] = getDistanceSensor("ds13");
    _dsensors[3] = getDistanceSensor("ds14");

    // Enable all necessaries distance sensors
    _dsensors[0]->enable(_time_step);
    _dsensors[1]->enable(_time_step);
    _dsensors[2]->enable(_time_step);
    _dsensors[3]->enable(_time_step);
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Disable all distance sensors on exit
    _dsensors[0]->disable();
    _dsensors[1]->disable();
    _dsensors[2]->disable();
    _dsensors[3]->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{ 
    // Variables to store the distance measured by de distance sensors
    double ds1_dist = 0, ds2_dist = 0, ds13_dist = 0, ds14_dist = 0;

    while (step(_time_step) != -1)
    {
        // Read the value of all distance sensors
        ds1_dist = _dsensors[0]->getValue();
        ds2_dist = _dsensors[1]->getValue();
        ds13_dist = _dsensors[2]->getValue();
        ds14_dist = _dsensors[3]->getValue();

        // Different robot states
        if (_robot_state == FREE)
            move_free(ds1_dist, ds14_dist);

        if (_robot_state == FOLLOWING_RIGHT_WALL)
            following_right_wall_state(ds1_dist, ds13_dist, ds14_dist);

        if (_robot_state == FOLLOWING_LEFT_WALL)
            following_left_wall_state(ds1_dist, ds2_dist, ds14_dist);

        // Call to the function that move the robot
        do_move();
    }
}

//////////////////////////////////////////////

void MyRobot::move_free(double ds1_dist, double ds14_dist)
{
    cout << "My state is FREE" << endl;
    if (ds1_dist > MAX_DISTANCE || ds14_dist > MAX_DISTANCE)
    {
        // The nearest wall is on the left
        if (ds1_dist > ds14_dist)
        {
            _robot_state = FOLLOWING_LEFT_WALL;
        }
        // The nearest wall is on the right
        else
        {
            _robot_state = FOLLOWING_RIGHT_WALL;
        }
    }
}

void MyRobot::following_right_wall_state(double ds1_dist, double ds13_dist, double ds14_dist)
{
    cout << "I am in \"FOLLOWING_RIGHT_WALL\"" << endl;
    if ((ds1_dist > MAX_DISTANCE) || (ds14_dist > MAX_DISTANCE)) {
        _mov_mode = BACK_RIGHT;
        cout << "Backing up and turning left." << endl;
    }
    else {
        if (ds13_dist > MAX_DISTANCE) {
            _mov_mode = TURN_LEFT;
            cout << "Turning left." << endl;
        }
        else {
            if (ds13_dist < MAX_DISTANCE + 50) {
                _mov_mode = TURN_RIGHT;
                cout << "Turning right." << endl;
            }
            else {
                _mov_mode = FORWARD;
                cout << "Moving forward." << endl;
            }
        }
    }
}

void MyRobot::following_left_wall_state(double ds1_dist, double ds2_dist, double ds14_dist)
{
    cout << "I am in \"FOLLOWING_LEFT_WALL\"" << endl;
    if ((ds1_dist > MAX_DISTANCE) || (ds14_dist > MAX_DISTANCE)) {
        _mov_mode = BACK_LEFT;
        cout << "Backing up and turning right." << endl;
    }
    else {
        if (ds2_dist > MAX_DISTANCE) {
            _mov_mode = TURN_RIGHT;
            cout << "Turning right." << endl;
        }
        else {
            if (ds2_dist < MAX_DISTANCE + 50) {
                _mov_mode = TURN_LEFT;
                cout << "Turning left." << endl;
            }
            else {
                _mov_mode = FORWARD;
                cout << "Moving forward." << endl;
            }
        }
    }
}

void MyRobot::do_move()
{
    // Send actuators commands according to the mode
    switch (_mov_mode)
    {
    case STOP:
        _left_speed = 0;
        _right_speed = 0;
        break;
    case FORWARD:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;
    case TURN_LEFT:
        _left_speed = MAX_SPEED / 1.25;
        _right_speed = MAX_SPEED;
        break;
    case TURN_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.25;
        break;
    case BACK_RIGHT:
        _left_speed = -MAX_SPEED / 3.0;
        _right_speed = -MAX_SPEED / 20.0;
        break;
    case BACK_LEFT:
        _left_speed = -MAX_SPEED / 20.0;
        _right_speed = -MAX_SPEED / 3.0;
        break;
    default:
        break;
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
