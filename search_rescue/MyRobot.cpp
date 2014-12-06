/**
 * @file    MyRobot.cpp
 * @brief   Full final proyect that makes a robot save 2 people from a hazard
 *
 * @author  Jesús Urdiales de la Parra          <100291428@alumnos.uc3m.es>
 * @author  Guillermo Mendiguchia Martinez      <100284346@alumnos.uc3m.es>
 * @date    2014-12-05
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;
    _step_control = 0;
    _left_speed = 0;
    _right_speed = 0;
    _persons_rescued = 0;
    _right_encoder = _left_encoder = 0.0;

    // Simulation control
    _simulation_time = 0;

    // Position inicialization
    _x = _y = _theta = 0.0;
    _prev_x = _prev_y = _prev_theta = 0.0;
    _theta_increment = 0.0;
    _person_x = _person_y = 0.0;

    // Cameras
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

    // The robot doesn't need spherical camera always enabled, so we don't enable it
    _spherical_camera = getCamera("camera_s");

    // Initial movement and state mode
    _mov_mode = FORWARD;
    _robot_state = STARTING;

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

    enableEncoders(_time_step);
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

    _forward_camera->disable();
    _spherical_camera->disable();
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // Variable where the robot stores the compass angle
    double compass_angle = 0.0;

    // Variables to store the distance measured by de distance sensors
    double ds_values[16];

    bool firstTime = true;
    double output_fcamera[2];
    int numRan = (int)(rand()%10 + 1 <= 5);

    while (step(_time_step) != -1)
    {
        // Read the value of all distance sensors and the compass
        read_distance_sensors(ds_values);
        const double *compass_val = _objetive_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        calculate_actual_position();
        forward_camera_control(output_fcamera);

        // Robot state control
        if (_robot_state == STARTING)
            start_move(compass_angle, ds_values);

        if (_robot_state == FOLLOWING_OBJETIVE)
            following_objetive_state(ds_values, compass_angle);

        if (_robot_state == FOLLOWING_RIGHT_WALL)
            following_right_wall_state(ds_values);

        if (_robot_state == FOLLOWING_LEFT_WALL)
            following_left_wall_state(ds_values);

        if (_robot_state == PERSON_FOUND)
        {
            if (_step_control == 0) approach_person(output_fcamera, ds_values, numRan);
            if (_step_control == 1) firstTime = wait_time(2, firstTime);
            if (_step_control == 2) firstTime = turn_left(360, firstTime);
            if (_step_control == 3)
            {
                firstTime = turn_left(180, firstTime);
                cout << "Person rescued, going away" << endl;
            }
            if (_step_control == 4)
            {
                _step_control = 0;
                _robot_state = FOLLOWING_OBJETIVE;
                numRan = (int)(rand()%10 + 1 <= 5);
            }
        }

        // Call to the function that sets robot move after checking if the program has finished
        do_move();

        // Call to the function that ends the program
        if (end_program()) break;

    }
}

//////////////////////////////////////////////
///     OBSTACLE AVOIDANCE CONTROL         ///
//////////////////////////////////////////////

void MyRobot::following_objetive_state(double ds_values[], double compass_angle)
{
    // If the robot is too close to an obstacle, avoid it
    if (ds_values[1] > MAX_DISTANCE || ds_values[14] > MAX_DISTANCE)
    {
        // The nearest wall is on the left
        if (ds_values[1] > ds_values[14])
            _robot_state = FOLLOWING_LEFT_WALL;

        // The nearest wall is on the right
        else
            _robot_state = FOLLOWING_RIGHT_WALL;
        return;
    }

    if (ds_values[0] > SECURITY_DISTANCE || ds_values[15] > SECURITY_DISTANCE)
    {
        // The nearest wall is on the left
        if (ds_values[0] > ds_values[15])
            _robot_state = FOLLOWING_LEFT_WALL;

        // The nearest wall is on the right
        else
            _robot_state = FOLLOWING_RIGHT_WALL;
        return;
    }

    // Robot avoids too close left wall
    if (ds_values[3] > (7 * MAX_DISTANCE))
    {
        _mov_mode = TURN_RIGHT;
        return;
    }

    // Robot turns when too close right wall
    if (ds_values[12] > (7 * MAX_DISTANCE))
    {
        _mov_mode = TURN_LEFT;
        return;
    }

    // We need to avoid crossing yellow lines if the robot has found only one person
    if (_persons_rescued == 1)
    {
        if (ds_values[6] > MAX_DISTANCE || ds_values[7] > SECURITY_DISTANCE)
        {
            _mov_mode = STOP;
            _robot_state = FOLLOWING_LEFT_WALL;
            return;
        }

        if (ds_values[8] > SECURITY_DISTANCE || ds_values[9] > MAX_DISTANCE)
        {
            _mov_mode = STOP;
            _robot_state = FOLLOWING_RIGHT_WALL;
            return;
        }

        // The robot needs to know if there is a yellow line and if the yellow line is in the front or in the back side of the robot
        int line = found_yellow_line_side();
        // 0 -> Advance, 1-> retreat
        if (line == 0)
            // Move forward
            _mov_mode = FORWARD;
        else
        {
            if (line == 1)
                _mov_mode = BACK;
            else
                _mov_mode = FORWARD;
        }
        //If the robot gets lost, it begins to move to the starting position
        if (_x < 12.0)
            _persons_rescued = -1;
        return;
    }

    // The robot calculates compass angle according to the number of persons rescued
    int desired_angle = 0;
    // Go to the map end
    if (_persons_rescued == 0 || _persons_rescued == 1) desired_angle = 45;
    // Go home
    else desired_angle = -135;

    // The robot is going home
    if(desired_angle == -135)
    {
        if(compass_angle <= 45 && compass_angle > -133)
            _mov_mode = TURN_LEFT_NO_OBSTACLE;
        else
        {
            if(compass_angle < -137 && compass_angle >= -180)
                _mov_mode = TURN_RIGHT_NO_OBSTACLE;
            else
            {
                if(compass_angle <= 180 && compass_angle > 45)
                    _mov_mode = TURN_RIGHT_NO_OBSTACLE;
                else
                    _mov_mode = FORWARD;
            }
        }
    }
    // The robot is going to map end, searching for two people
    else
    {
        if (compass_angle >= -135 && compass_angle < 43)
            _mov_mode = TURN_RIGHT_NO_OBSTACLE;
        else
        {
            if (compass_angle < -135 && compass_angle >= -180)
                _mov_mode = TURN_LEFT_NO_OBSTACLE;
            else
            {
                if (compass_angle > 2 && compass_angle <= 180)
                    _mov_mode = TURN_LEFT_NO_OBSTACLE;
                else
                    _mov_mode = FORWARD;
            }
        }
    }
}

void MyRobot::following_right_wall_state(double ds_values[])
{
    // If the robot hasn't found a person and finds itself int he starting point, it turns around and keeps looking
    if (_persons_rescued == 0 && _x < 0 && ds_values[0] < MAX_DISTANCE && ds_values[15] < MAX_DISTANCE)
    {
        _mov_mode = TURN_LEFT_NO_OBSTACLE;
        return;
    }

    // We need to avoid crossing yellow lines if the robot has found only one person
    if (_persons_rescued == 1)
    {
        // The robot needs to know if there is a yellow line and if the yellow line is in the right or left side of the robot
        int line = found_yellow_line_side();
        // Left side  = 0, right side = 1, no line = -1
        if (line != -1)
        {
            _mov_mode = TURN_LEFT_STATIC;
            return;
        }

        // When going backwards, turn
        if (ds_values[7] > SECURITY_DISTANCE || ds_values[8] > SECURITY_DISTANCE)
        {
            if ( (ds_values[8] > ds_values[7]) && (ds_values[11] < SECURITY_DISTANCE) && (ds_values[14] < MAX_DISTANCE) )
                _mov_mode = TURN_RIGHT;

            if ( (ds_values[7] > ds_values[8]) && (ds_values[4] < SECURITY_DISTANCE) && (ds_values[1] < MAX_DISTANCE) )
                _mov_mode = TURN_LEFT;
            return;
        }

        if (_x < 12.0)
            _persons_rescued = -1;
    }

    // There is an obstacle in front of the robot
    if ((ds_values[1] > MAX_DISTANCE) || (ds_values[14] > MAX_DISTANCE) || ds_values[0] > SECURITY_DISTANCE || ds_values[15] > SECURITY_DISTANCE )
    {
        // The robot is about to avoid the wall completly
        if (ds_values[1] > MAX_DISTANCE && ds_values[2] > MAX_DISTANCE && ds_values[14] == 0 && ds_values[13] == 0 && ds_values[12] == 0)
        {
            _mov_mode = STOP;
            _robot_state = FOLLOWING_OBJETIVE;
            return;
        }

        // The robot can go back
        if (ds_values[12] < SECURITY_DISTANCE && ds_values[10] < SECURITY_DISTANCE && ds_values[8] < SECURITY_DISTANCE )
        {
            _mov_mode = BACK_RIGHT;
            cout << "Backing up and turning left." << endl;
        }
        else
        {
            if ( ds_values[12] > SECURITY_DISTANCE || ds_values[13] > SECURITY_DISTANCE || ds_values[14] > MAX_DISTANCE || ds_values[15] > SECURITY_DISTANCE )
            {
                _mov_mode = TURN_LEFT_STATIC;
                cout << "Turning left static" << endl;
            }
            else
            {
                _mov_mode = TURN_LEFT;
                cout << "Turning left, no static" << endl;
            }
        }

        return;
    }

    // With these conditions the robot avoids crashing with corners
    if ( ((ds_values[13] > (8 * MAX_DISTANCE) || ds_values[12] > (8 * MAX_DISTANCE)) ) && ds_values[13] > ds_values[12] )
    {
        // The robot should check if there is an obtacle behind it
        if (ds_values[6] > SECURITY_DISTANCE || ds_values[9] > SECURITY_DISTANCE)
            _mov_mode = TURN_LEFT_STATIC;
        else
            _mov_mode = BACK_RIGHT;
        cout << "Avoiding corners" << endl;
        return;
    }

    // The robot is too close to a wall
    if (ds_values[13] > MAX_DISTANCE)
    {
        _mov_mode = TURN_LEFT;
        cout << "Turning left." << endl;
        return;
    }

    // The robot knows when the obstacle has been passed
    if ((ds_values[12] == 0) && (ds_values[11] == 0) && (ds_values[13] == 0))
    {
        _mov_mode = STOP;
        // If the robot has found one person it needs to search for another, it doesn't need to follow the compass
        _robot_state = FOLLOWING_OBJETIVE;
        cout << "Mode changed to following objective." << endl;
        return;
    }

    if (ds_values[13] < (MAX_DISTANCE + 50))
    {
        _mov_mode = TURN_RIGHT;
        cout << "Turning right." << endl;
        return;
    }

    _mov_mode = FORWARD;
    cout << "Moving forward." << endl;
}

void MyRobot::following_left_wall_state(double ds_values[])
{       //If the robot hasn't found a person and finds itself int he starting point, it turns around and keeps looking
    if (_persons_rescued == 0 && _x < 0 && ds_values[0] < MAX_DISTANCE && ds_values[15] < MAX_DISTANCE)
    {
        _mov_mode = TURN_RIGHT_NO_OBSTACLE;
        return;
    }

    // We need to avoid crossing yellow lines if the robot has found only one person
    if (_persons_rescued == 1)
    {
        // The robot needs to know if there is a yellow line and if the yellow line is in the left or right side of the robot
        int line = found_yellow_line_side();
        // Left side  = 0, right side = 1
        if (line != -1)
        {
            _mov_mode = TURN_RIGHT_STATIC;
            return;
        }

        // When going backwards, turn
        if (ds_values[7] > SECURITY_DISTANCE || ds_values[8] > SECURITY_DISTANCE)
        {
            if ( (ds_values[8] > ds_values[7]) && (ds_values[11] < SECURITY_DISTANCE) && (ds_values[14] < MAX_DISTANCE) )
                _mov_mode = TURN_RIGHT;

            if ( (ds_values[7] > ds_values[8]) && (ds_values[4] < SECURITY_DISTANCE) && (ds_values[1] < MAX_DISTANCE) )
                _mov_mode = TURN_LEFT;
            return;
        }

        if (_x < 12.0)
            _persons_rescued = -1;
    }

    // There is an obstacle in front of the robot
    if ((ds_values[1] > MAX_DISTANCE) || (ds_values[14] > MAX_DISTANCE) || ds_values[0] > SECURITY_DISTANCE || ds_values[15] > SECURITY_DISTANCE)
    {
        // The robot is about to avoid the wall completely
        if (ds_values[14] > MAX_DISTANCE && ds_values[13] > MAX_DISTANCE && ds_values[1] == 0 && ds_values[2] == 0 && ds_values[3] == 0)
        {
            _mov_mode = STOP;
            _robot_state = FOLLOWING_OBJETIVE;
            cout << "Change to following objetive." << endl;
            return;
        }

        // The robot can go back
        if (ds_values[3] < SECURITY_DISTANCE && ds_values[5] < SECURITY_DISTANCE && ds_values[7] < SECURITY_DISTANCE)
        {
            _mov_mode = BACK_LEFT;
            cout << "Backing up and turning right." << endl;
            return;
        }
        else
        {
            if (ds_values[3] > SECURITY_DISTANCE || ds_values[2] > SECURITY_DISTANCE || ds_values[1] > MAX_DISTANCE || ds_values[0] > SECURITY_DISTANCE)
            {
                _mov_mode = TURN_RIGHT_STATIC;
                cout << "Turning right static" << endl;
            }
            else
            {
                _mov_mode = TURN_RIGHT;
                cout << "Turning right, no static" << endl;
            }
        }

        return;
    }

    // With these conditions the robot avoids crashing with corners
    if ( ((ds_values[2] > (8 * MAX_DISTANCE) || ds_values[3] > (8 * MAX_DISTANCE))) && ds_values[2] > ds_values[3] )
    {
        if (ds_values[6] > SECURITY_DISTANCE || ds_values[9] > SECURITY_DISTANCE)
            _mov_mode = TURN_RIGHT_STATIC;
        else
            _mov_mode = BACK_LEFT;
        cout << "Avoiding corners" << endl;
        return;
    }

    // The robot is too close to a wall
    if (ds_values[2] > MAX_DISTANCE)
    {
        _mov_mode = TURN_RIGHT;
        cout << "Turning right." << endl;
        return;
    }

    // The robot knows when the obstacle has been passed
    if ((ds_values[3] == 0) && (ds_values[4] == 0) && ds_values[2] == 0)
    {
        // If the robot has found one person it needs to search for another, it doesn't need to follow the compass
        _robot_state = FOLLOWING_OBJETIVE;
        _mov_mode = STOP;
        cout << "Mode changed to following objetive" << endl;
        return;
    }

    if ((ds_values[2] < MAX_DISTANCE + 50))
    {
        _mov_mode = TURN_LEFT;
        cout << "Turning left." << endl;
        return;
    }

    _mov_mode = FORWARD;
}

//////////////////////////////////////////////
////          POSITION CONTROL            ////
//////////////////////////////////////////////

void MyRobot::calculate_actual_position()
{
    // Read the enconders
    _left_encoder = getLeftEncoder();
    _right_encoder = getRightEncoder();

    // Calculate wheels displacement
    double dr = (_right_encoder / ENCODER_RESOLUTION) * WHEEL_RADIUS;
    double dl = (_left_encoder / ENCODER_RESOLUTION) * WHEEL_RADIUS;

    // It is necessary to reset the encoders because the next calculations operate with increments
    setEncoders (0,0);

    // Calculate actual robot position
    _x = _x + ((dr + dl) / 2) * cos(_theta + (dr - dl) / (2 * DISTANCE_B_WHEELS));
    _y = _y + ((dr + dl) / 2) * sin(_theta + (dr - dl) / (2 * DISTANCE_B_WHEELS));
    _theta = _theta + (dr - dl) / DISTANCE_B_WHEELS;

    // We need theta in range (-180, 180] to ensure the correct operation of the control
    if (_theta > M_PI)
        _theta = - M_PI + (_theta - M_PI);
    if (_theta <= - M_PI)
        _theta = M_PI - (- M_PI - _theta);

}

//////////////////////////////////////////////
////          MOVEMENT CONTROL           /////
//////////////////////////////////////////////

void MyRobot::start_move(double compass_angle, double ds_values[] )
{
    // Iniciating theta to its true value (0º corresponds at X axe)
    _theta = (compass_angle * M_PI) / 180 - M_PI / 4;

    // If the robot starts facing a wall very closely, it moves backwards a short distance
    if (ds_values [0] > MAX_DISTANCE || ds_values[15] > MAX_DISTANCE)
    {
        _mov_mode = BACK;
        return;
    }
    // If the robots is facing the compass direction, advances
    if ( (compass_angle > (DESIRED_ANGLE - 3)) && (compass_angle < (DESIRED_ANGLE + 3)) )
        _robot_state = FOLLOWING_OBJETIVE;

    if ( (compass_angle < DESIRED_ANGLE) && (compass_angle >= -135) )
        _mov_mode = TURN_RIGHT_STATIC;
    else
        _mov_mode = TURN_LEFT_STATIC;
}

bool MyRobot::end_program()
{
    // The robot need to know when it has finished the program
    if ( (_persons_rescued == 2 || _persons_rescued == -1) && (_x < 0.5) )
    {
        if (_persons_rescued == 2)
            cout << "I have rescued two people, finishing program." << endl;
        else
            cout << "I could only rescue one person, finishing program." << endl;
        setSpeed(0.0, 0.0);
        return true;
    }
    else
        return false;
}

bool MyRobot::turn_left(double turn_deg, bool first_time)
{
    // The robots spins until it has turned for a desired angle
    if (first_time == true)
    {
        _mov_mode = STOP;
        _theta_increment = 0.0;
        first_time = false;
        _prev_theta = _theta;
    }
    else
    {
        double turn_rad = turn_deg * M_PI / 180;

        // Calculate theta increment
        double aux = std::abs(_prev_theta - _theta);

        if ( aux > M_PI)
            _theta_increment = _theta_increment + (2 * M_PI - aux);
        else
            _theta_increment = _theta_increment + std::abs(_prev_theta - _theta);

        _prev_theta = _theta;

        if (_theta_increment >= turn_rad)
        {
            _mov_mode = STOP;
            _theta_increment = _prev_theta = 0.0;
            _step_control++;
            return true;
        }
        else
        {
            _mov_mode = TURN_FAST;
        }
    }
    return false;
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
        _left_speed = MAX_SPEED / 1.30;
        _right_speed = MAX_SPEED;
        break;
    case TURN_LEFT_STATIC:
        _left_speed = - MAX_SPEED / 7.0;
        _right_speed = MAX_SPEED / 7.0;
        break;
    case TURN_LEFT_NO_OBSTACLE:
        _left_speed = MAX_SPEED / 1.60;
        _right_speed = MAX_SPEED;
        break;
    case TURN_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.30;
        break;
    case TURN_RIGHT_STATIC:
        _left_speed = MAX_SPEED / 7.0;
        _right_speed = - MAX_SPEED / 7.0;
        break;
    case TURN_RIGHT_NO_OBSTACLE:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.60;
        break;
    case APPROACH_LEFT:
        _left_speed = MAX_SPEED / 1.18;
        _right_speed = MAX_SPEED;
        break;
    case APPROACH_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.18;
        break;
    case BACK:
        _left_speed = - MAX_SPEED;
        _right_speed = - MAX_SPEED;
        break;
    case BACK_RIGHT:
        _left_speed = -MAX_SPEED / 3.0;
        _right_speed = -MAX_SPEED / 20.0;
        break;
    case BACK_LEFT:
        _left_speed = -MAX_SPEED / 20.0;
        _right_speed = -MAX_SPEED / 3.0;
        break;
    case TURN_FAST:
        _left_speed = -MAX_SPEED / 4.0;
        _right_speed = MAX_SPEED / 4.0;
    default:
        break;
    }

    // Set the motor speeds
    setSpeed(_left_speed, _right_speed);
}

//////////////////////////////////////////////
///            CAMERAS CONTROL             ///
//////////////////////////////////////////////

void MyRobot::forward_camera_control(double output_fcamera[])
{
    int green_pixels = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_green = 0.0;

    // Get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    int min_x = image_width_f, max_x = 0;

    // Get current image from forward camera
    const unsigned char *image_f = _forward_camera->getImage();

    // count number of pixels that are green
    for (int x = 0; x < image_width_f; x++)
    {
        for (int y = 0; y < image_height_f; y++)
        {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            // Components for green pixels
            if ( (red < 114) && (green > 50) && (blue < 114) && (red != green) && (green != blue) && (blue != red) )
            {
                green_pixels++;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
            }
        }
    }

    // We always have to calculate green percentaje
    percentage_green = (green_pixels / (float) (image_width_f * image_height_f)) * 100;

    // The robot determines if it has found a person (except in the case the robot has found two people)
    if( (percentage_green > 2) && (_robot_state != PERSON_FOUND) && _persons_rescued != 2 && _persons_rescued != -1 )
    {
        cout << "Person " << _persons_rescued + 1 << " found" << endl;
        _robot_state = PERSON_FOUND;
        _mov_mode = STOP;
    }

    // The robot need to know where is (angle) the person
    if (_robot_state == PERSON_FOUND && _step_control == 0)
    {
        double ang = 0.0;
        double pos_person = (max_x + min_x) / 2;

        // If the person is on the left side
        if (pos_person < image_width_f / 2)
            ang = _theta + (image_width_f / 2 - pos_person) * (M_PI / (2 * image_width_f));
        // Right side
        else
            ang = _theta - (pos_person - image_width_f / 2) * (M_PI / (2 * image_width_f));

        output_fcamera[0] = percentage_green;
        output_fcamera[1] = ang;
    }
    else
    {
        output_fcamera[0] = percentage_green;
        output_fcamera[1] = 0.0;
    }
}

int MyRobot::found_yellow_line_side()
{
    int sum = 0, yellow_pixels_bottom = 0, yellow_pixels_top = 0;
    double percentage_yellow = 0.0;
    unsigned char green = 0, red = 0, blue = 0;

    // Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();

    // Get current image from forward camera
    const unsigned char *image_s = _spherical_camera->getImage();

    for (int x = 0; x < image_width_s; x++)
    {
        for (int y = 0; y < image_height_s; y++)
        {
            green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
            red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
            blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

            // The robot determines the location(top or bottom) of the yellow line
            if ( (red > 100) && (green > 100) && (blue < 142) && (abs(red - blue) > 50) )
            {
                sum++;
                if (y < (int)image_height_s / 2)
                    yellow_pixels_top++;
                else
                    yellow_pixels_bottom++;
            }
        }
    }

    // Percentage of yellow in spherical camera
    percentage_yellow = sum / (float)(image_height_s * image_width_s) * 100;
    cout << "per yellow: " << percentage_yellow << endl;
    double threshold = 0.0;

    if (_robot_state == FOLLOWING_RIGHT_WALL || _robot_state == FOLLOWING_LEFT_WALL)
        threshold = 0.8;
    else
        threshold = 1.2;

    if (percentage_yellow > threshold)
    {
        // Position of the yellow line
        if (yellow_pixels_top < yellow_pixels_bottom)
            // Advance-> the robot must turn right to avoid the yellow line
            return 0;
        else
            // Retreat
            return 1;
    }
    else
        // No line detected
        return -1;
}

//////////////////////////////////////////////
///     CONTROL FOR RESCUE PERSONS         ///
//////////////////////////////////////////////

// We use odometry in this function, so we will work with radians
void MyRobot::approach_person(double output_fcamera[], double ds_values[], int numRand)
{
    cout << "Rescue person state" << "\tgreen percentage: " << output_fcamera[0] << endl;

    // If we have found a person and we see the same person, we go away
    if (_persons_rescued == 1 && output_fcamera[0] > GREEN_PERCENTAGE_FOUND)
    {
        // The robot compares its position with the position of the last person found
        if (( (_x < _person_x + 1.2) && (_x > _person_x - 1.2)) && ((_y < _person_y + 1.2) && (_y > _person_y - 1.2)))
        {
            cout << "I have seen this person before" << endl;
            if (output_fcamera[0] == 0)
            {
                _robot_state = FOLLOWING_OBJETIVE;
                _step_control = 0;
                _mov_mode = STOP;
            }
            // We turn randomly to a direction to avoid the person
            else
            {
                if (numRand <= 5)
                    _mov_mode = TURN_RIGHT_STATIC;
                else
                    _mov_mode = TURN_LEFT_STATIC;
            }
            return;
        }
    }

    // The robot must check if there is an obstacle in front of it
    if (ds_values[1] > SECURITY_DISTANCE || ds_values[2] > SECURITY_DISTANCE)
    {
        if (ds_values[7] < SECURITY_DISTANCE || ds_values[8] < SECURITY_DISTANCE)
            _mov_mode = BACK_LEFT;
        else
            _mov_mode = TURN_RIGHT_STATIC;
        return;
    }

    if (ds_values[14] > SECURITY_DISTANCE || ds_values[13] > SECURITY_DISTANCE)
    {
        if (ds_values[7] < SECURITY_DISTANCE || ds_values[8] < SECURITY_DISTANCE)
            _mov_mode = BACK_RIGHT;
        else
            _mov_mode = TURN_LEFT_STATIC;
        return;
    }

    // The robot should approach the person, avoiding posible obstacles
    if(output_fcamera[0] < GREEN_PERCENTAGE_FOUND)
    {
        // Due to an error the robot has lost the view of the person. It has to exit this mode
        if (output_fcamera[0] == 0)
        {
            _mov_mode = STOP;
            _robot_state = FOLLOWING_OBJETIVE;
            _step_control = 0;
            cout << "I have lost sight of the person" << endl;
        }

        cout << "Position of the person: " << output_fcamera[1] * M_PI / 180 << " degrees" << endl;

        // The robot follows the angle passed in function arguments (error equivalent to 2 degrees)
        // We use approach movements because they are smoother
        if (_theta < (output_fcamera[1] - 0.035))
            _mov_mode = APPROACH_LEFT;
        else
        {
            if (_theta > (output_fcamera[1] + 0.035))
                _mov_mode = APPROACH_RIGHT;
            else
                _mov_mode = FORWARD;
        }
        return;
    }

    // When the distance between the robot and the person is less than one meter it stops
    _mov_mode = STOP;
    _step_control = 1;
}

bool MyRobot::wait_time(double seconds, bool firstTime)
{
    if (firstTime == true)
    {
        _mov_mode = STOP;
        _simulation_time = getTime();
        firstTime = false;
    }

    if ((getTime() - _simulation_time) < seconds)
    {
        _mov_mode = STOP;
        firstTime = false;
    }
    else
    {
        double alpha = 0;
        _persons_rescued++;
        // When the robot has waited for desired seconds it saves actual person position for the first rescued person
        if (_persons_rescued == 1)
        {
            // We fix the position of the person coming from every cuadrant
            if (0 <=_theta && _theta <= M_PI / 2)
            {
                _person_x = _x + cos(_theta);
                _person_y = _y + sin(_theta);
            }
            else
            {
                if(M_PI / 2 < _theta && _theta < M_PI)
                {
                    alpha = M_PI - _theta;
                    _person_x = _x - cos(alpha);
                    _person_y = _y + sin(alpha);
                }
                else
                {
                    if(_theta < 0 && - 90 < _theta)
                    {
                        _person_x = _x + cos(-_theta);
                        _person_y = _y - sin(-_theta);
                    }
                    else
                    {
                        alpha = M_PI + _theta;
                        _person_x = _x + cos(alpha);
                        _person_y = _y - sin(alpha);
                    }
                }
            }
            // The robot need to enable spherical camera to see yellow lines
            _spherical_camera->enable(_time_step);
        }
        if (_persons_rescued == 2)
            // The robot does not need images from spherical camera anymore
            _spherical_camera->disable();

        _step_control = 2;
        // On function exit variable firstTime has to be true
        firstTime = true;
    }

    return firstTime;
}

//////////////////////////////////////////////
///             SENSORS MEASUREMENTS       ///
//////////////////////////////////////////////

void MyRobot::read_distance_sensors(double *ds_values)
{
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
}

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}
