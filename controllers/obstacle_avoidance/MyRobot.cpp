/**
 * @file    MyRobot.cpp
 * @brief   A program that shows how the robot avoids obstacles.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Initial movement and state mode
    _mov_mode = FORWARD;
    _robot_state = FOLLOWING_OBJETIVE;

    //Get and enable the compass
    _objetive_compass = getCompass("compass");
    _objetive_compass->enable(_time_step);

    // Get the distance sensors
    _dsensors[0] = getDistanceSensor("ds0");
    _dsensors[1] = getDistanceSensor("ds1");
    _dsensors[2] = getDistanceSensor("ds2");
    _dsensors[3] = getDistanceSensor("ds3");
    _dsensors[4] = getDistanceSensor("ds4");
    _dsensors[5] = getDistanceSensor("ds5");
    _dsensors[6] = getDistanceSensor("ds6");
    _dsensors[7] = getDistanceSensor("ds7");
    _dsensors[8] = getDistanceSensor("ds8");
    _dsensors[9] = getDistanceSensor("ds9");
    _dsensors[10] = getDistanceSensor("ds10");
    _dsensors[11] = getDistanceSensor("ds11");
    _dsensors[12] = getDistanceSensor("ds12");
    _dsensors[13] = getDistanceSensor("ds13");
    _dsensors[14] = getDistanceSensor("ds14");
    _dsensors[15] = getDistanceSensor("ds15");

    // Enable all distance sensors
    for (int i = 0; i < NUMBER_OF_DSENSORS; i++)
    {
        _dsensors[i]->enable(_time_step);
    }
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Disable all devices
    _objetive_compass->disable();
    for (int i = 0; i<NUMBER_OF_DSENSORS; i++)
    {
        _dsensors[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // Variable where the robot store compass angle
    double compass_angle = 0.0;
    // Variables to store the distance measured by de distance sensors
    double ds_values[16];

    while (step(_time_step) != -1)
    {
        // Read the value of all distance sensors and the compass
        ds_values[0] = _dsensors[0]->getValue();
        ds_values[1] = _dsensors[1]->getValue();
        ds_values[2] = _dsensors[2]->getValue();
        ds_values[3] = _dsensors[3]->getValue();
        ds_values[4] = _dsensors[4]->getValue();
        ds_values[5] = _dsensors[5]->getValue();
        ds_values[6] = _dsensors[6]->getValue();
        ds_values[7] = _dsensors[7]->getValue();
        ds_values[8] = _dsensors[8]->getValue();
        ds_values[9] = _dsensors[9]->getValue();
        ds_values[10] = _dsensors[10]->getValue();
        ds_values[11] = _dsensors[11]->getValue();
        ds_values[12] = _dsensors[12]->getValue();
        ds_values[13] = _dsensors[13]->getValue();
        ds_values[14] = _dsensors[14]->getValue();
        ds_values[15] = _dsensors[15]->getValue();
        const double *compass_val = _objetive_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Initial mode, the robot just go forward following campass direction
        if (_robot_state == FOLLOWING_OBJETIVE)
            following_objetive_state(ds_values, compass_angle);

        if (_robot_state == FOLLOWING_RIGHT_WALL)
            following_right_wall_state(ds_values);

        if (_robot_state == FOLLOWING_LEFT_WALL)
            following_left_wall_state(ds_values);

        // Call to the function that set robot move
        do_move();
    }
}

//////////////////////////////////////////////

void MyRobot::following_objetive_state(double ds_values[], double compass_angle)
{
    // If the robot is too close to an obstacle, avoid it
    if (ds_values[1] > MAX_DISTANCE || ds_values[14] > MAX_DISTANCE)
    {
        // The nearest wall is on the left
        if (ds_values[1] > ds_values[14])
        {
            _robot_state = FOLLOWING_LEFT_WALL;
        }
        // The nearest wall is on the right
        else
        {
            _robot_state = FOLLOWING_RIGHT_WALL;
        }
    }
    // The robot follows compass direction
    else
    {
        // I am going straight to my objetive
        if (compass_angle < (DESIRED_ANGLE - 2))
            _mov_mode = TURN_RIGHT;
        else
        {
            if (compass_angle > (DESIRED_ANGLE + 2))
                _mov_mode = TURN_LEFT;
            else
                _mov_mode = FORWARD;
        }
    }
}

void MyRobot::following_right_wall_state(double ds_values[])
{
    cout << "I am in \"FOLLOWING_RIGHT_WALL\"" << endl;
    if ((ds_values[1] > MAX_DISTANCE) || (ds_values[14] > MAX_DISTANCE)) {
        _mov_mode = BACK_RIGHT;
        cout << "Backing up and turning left." << endl;
    }
    else {
        if (ds_values[13] > MAX_DISTANCE) {
            _mov_mode = TURN_LEFT;
            cout << "Turning left." << endl;
        }
        else {
            if ((ds_values[12] == 0) && (ds_values[11] == 0)) {
                _mov_mode = TURN_RIGHT;
                _robot_state = FOLLOWING_OBJETIVE;
                cout << "Turning right." << endl;
            }
            else {
                if (ds_values[13] < (MAX_DISTANCE + 50)) {
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
}

void MyRobot::following_left_wall_state(double ds_values[])
{
    cout << "I am in \"FOLLOWING_LEFT_WALL\"" << endl;
    if ((ds_values[1] > MAX_DISTANCE) || (ds_values[14] > MAX_DISTANCE)) {
        _mov_mode = BACK_LEFT;
        cout << "Backing up and turning left." << endl;
    }
    else {
        if (ds_values[2] > MAX_DISTANCE) {
            _mov_mode = TURN_RIGHT;
            cout << "Turning right." << endl;
        }
        else {
            if ((ds_values[3] == 0) && (ds_values[4] == 0)) {
                _robot_state = FOLLOWING_OBJETIVE;
                _mov_mode = TURN_LEFT;
                cout << "Turning left." << endl;
            }
            else {
                if ((ds_values[2] < MAX_DISTANCE + 50)) {
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
        _left_speed = MAX_SPEED / 1.50;
        _right_speed = MAX_SPEED;
        break;
    case TURN_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.50;
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
