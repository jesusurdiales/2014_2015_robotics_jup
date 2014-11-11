/**
 * @file    MyRobot.cpp
 * @brief   The robot avoids an obstacle only using odometry.
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

    _robot_actual_mode = CALCULATING_NEXT_POINT;
    _x = 0.0, _y = 0.0, _theta = 0.0;

    _distance_to_objetive = 0.0;
    _angle_to_objetive = 0.0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int next_point = 1;
    double partial_distance = 0.0;
    double *pd = &partial_distance;

    while (step(_time_step) != -1)
    {
        // First the robot needs to know its actual position
        calculate_actual_position(pd);

        // Then the robot checks its actual state
        if (_robot_actual_mode == CALCULATING_NEXT_POINT)
        {
            // If true, the program has ended
            if ( points_to_reach(next_point) ) break;
            next_point++;
            continue;
        }

        if (_robot_actual_mode == TURNING_RIGHT)
            turn_right();
        if (_robot_actual_mode == TURNING_LEFT)
            turn_left();
        if (_robot_actual_mode == GO_AHEAD)
            go_ahead(pd);
    }
}

//////////////////////////////////////////////

bool MyRobot::points_to_reach(int next_point)
{
    switch (next_point)
    {
    case 1:
        set_next_point(5, -4);
        break;
    case 2:
        set_next_point(12.5, -4);
        break;
    case 3:
        set_next_point(18, 0);
        break;
    default:
        cout << "End of program, the robot has arrived to the map end." << endl;
        return true;
    }

    cout << "Next point to reach: " << next_point << endl;
    return false;
}

void MyRobot::go_ahead(double *pdistance)
{
    // First the robot checks if it has reached its objetive
    if  ( *pdistance >= _distance_to_objetive )
    {
        _robot_actual_mode = CALCULATING_NEXT_POINT;
        setSpeed (0,0);
        *pdistance = 0.0;
        _distance_to_objetive = 0.0;
        return;
    }

    // The robot tries to move towards the target in a straight line. 0.035 rads ~ 2 deg
    if (_theta > (_angle_to_objetive + 0.035))
    {
        // Robot must turn right
        cout << "I am turning right"  << endl;
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.25;
        setSpeed (_left_speed, _right_speed);
        return;
    }

    if (_theta < (_angle_to_objetive - 0.035))
    {
        // Robot must turn left
        cout << "I am turning left"  << endl;
        _left_speed = MAX_SPEED / 1.25;;
        _right_speed = MAX_SPEED;
        setSpeed (_left_speed, _right_speed);
        return;
    }

    cout << "I am running forward" << endl;
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
    setSpeed (_left_speed, _right_speed);
}

void MyRobot::turn_right()
{
    if (_theta <= _angle_to_objetive)
    {
        setSpeed (0.0, 0.0);
        _robot_actual_mode = GO_AHEAD;
    }
    else
    {
        // Robot must turn right
        cout << "I am turning right"  << endl;
        _left_speed = MAX_SPEED / 2;
        _right_speed = 0.0;
        setSpeed (_left_speed, _right_speed);
    }
}

void MyRobot::turn_left()
{
    if (_theta >= _angle_to_objetive)
    {
        setSpeed (0.0, 0.0);
        _robot_actual_mode = GO_AHEAD;
    }
    else
    {
        // Robot must turn left
        cout << "I am turning left"  << endl;
        _left_speed = 0.0;
        _right_speed = MAX_SPEED / 2;
        setSpeed (_left_speed, _right_speed);
    }
}

void MyRobot::calculate_actual_position(double *pdistance)
{
    // Read the enconders
    _left_encoder = getLeftEncoder();
    _right_encoder = getRightEncoder();

    // Calculate wheels displacement
    double dr = (_right_encoder / ENCODER_RESOLUTION) * WHEEL_RADIUS;
    double dl = (_left_encoder / ENCODER_RESOLUTION) * WHEEL_RADIUS;
    // It is necessary to reset the encoders because next calculations operate with increments
    setEncoders (0,0);

    // Calculate actual robot position and distance from the last point
    _x = _x + ((dr + dl) / 2) * cos(_theta + (dr - dl) / (2 * DISTANCE_B_WHEELS));
    _y = _y + ((dr + dl) / 2) * sin(_theta + (dr - dl) / (2 * DISTANCE_B_WHEELS));
    _theta = _theta + (dr - dl) / DISTANCE_B_WHEELS;
    *pdistance = *pdistance + (dr + dl) / 2;

    // Show actual position
    cout << "x = " << _x << ", y = " << _y << ", theta = " << _theta << endl;
    cout << "Distance to next point = " << _distance_to_objetive - *pdistance << endl;
}

void MyRobot::set_next_point(double xx, double yy)
{
    // Calculate angle to objetive
    _angle_to_objetive = normalize_angle (calculate_absolute_angle(_x, _y, xx, yy));
    // Distance between two points
    _distance_to_objetive = sqrt( (xx - _x)*(xx - _x) + (yy - _y)*(yy - _y) );
    cout << "angle: " << _angle_to_objetive << ", distance to objetive: " << _distance_to_objetive << endl << endl;

    // The robot must to know if it is necessary to turn right or left
    if (_theta > _angle_to_objetive)
    {
        _robot_actual_mode = TURNING_RIGHT;
    }
    else
    {
        _robot_actual_mode = TURNING_LEFT;
    }
}

double MyRobot::normalize_angle(double angle)
{
    if (angle > -M_PI && angle <= M_PI)
        return angle;

    while (angle <= -M_PI)
        angle = angle + 2*M_PI;
    while (angle > M_PI)
        angle = angle - 2*M_PI;
    return angle;
}

double MyRobot::calculate_absolute_angle(double xi, double yi, double xf, double yf)
{
    double xo = xf - xi;
    double yo = yf - yi;
    double angle = atan( (yf - yi) / (xf - xi) );

    // Is necessary to
    if (xo < 0 && yo > 0)
    {
        return (angle + M_PI);
    }

    if (xo < 0 && yo <0 )
    {
        return (angle + M_PI);
    }

    if (xo > 0 && yo < 0)
    {
        return (angle + 2*M_PI);
    }
    return angle;
}
