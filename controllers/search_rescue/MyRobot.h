/**
 * @file    MyRobot.h
 * @brief   Full final proyect that makes a robot save 2 people from a hazard
 *
 * @author  Jesús Urdiales de la Parra          <100291428@alumnos.uc3m.es>
 * @author  Guillermo Mendiguchia Martinez      <100284346@alumnos.uc3m.es>
 * @date    2014-12-05
 */

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUMBER_OF_DSENSORS      16
#define MAX_DISTANCE            100
#define MAX_SPEED               80
#define DESIRED_ANGLE           45
#define ENCODER_RESOLUTION      5
#define WHEEL_RADIUS            0.0825
#define DISTANCE_B_WHEELS       0.32
#define GREEN_PERCENTAGE_FOUND  40
#define SECURITY_DISTANCE       500

class MyRobot : public DifferentialWheels
{
private:
    int _time_step;
    int _step_control;

    // Declaration of all devices
    Camera *_forward_camera;
    Camera *_spherical_camera;
    Compass *_objetive_compass;
    DistanceSensor * _dsensors[NUMBER_OF_DSENSORS];

    // Different states or behaviours of the robot
    enum State
    {
        STARTING, FOLLOWING_OBJETIVE, FOLLOWING_RIGHT_WALL, FOLLOWING_LEFT_WALL, PERSON_FOUND
    };
    State _robot_state;

    // Speed control
    double _left_speed, _right_speed;

    // Declaration of all movement modes
    enum Mode
    {
        STOP, TURN_RIGHT, TURN_RIGHT_STATIC, TURN_LEFT, TURN_LEFT_STATIC, FORWARD, BACK_RIGHT, BACK_LEFT,
        APPROACH_RIGHT, APPROACH_LEFT, TURN_LEFT_NO_OBSTACLE, TURN_RIGHT_NO_OBSTACLE, BACK, TURN_FAST
    };
    Mode _mov_mode;

    // Persons variables
    int _persons_rescued;
    double _person_x;
    double _person_y;

    // Actual robot coordinates
    double _x;
    double _y;
    double _theta;

    // Previous robot coordinates
    double _prev_x;
    double _prev_y;
    double _prev_theta;

    double _theta_increment;
    double _left_encoder, _right_encoder;
    double _simulation_time;

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
     * @return The angle of the compass
     */
    double convert_bearing_to_degrees(const double* in_vector);

    /**
     * @brief Begins the movement of the robot
     * @param compass_angle The angle which we are wanting to direct the robot
     * @param *ds_values Array of doubles that contains the distance sensors values.
     */
    void start_move(double compass_angle,double *ds_values );

    /**
     * @brief Moves towards a person
     * @param output_fcamera[] This array contains the amount of green and the angle between the camera and the person
     * @param ds_values[] Array of doubles that contains old distance sensors values.
     * @param numRand The random number for which we decide where to turn
     */
    void approach_person(double output_fcamera[], double ds_values[], int numRand);

    /**
     * @brief This function controls the frontal camera
     * param output_fcamera[] Array that shows us the percentage of green and the angle of the person we have found
     */
    void forward_camera_control(double output_fcamera[]);

    /**
     * @brief Detects the yellow line where the 2 persons are located
     * @return 0,1 or -1 to see if we have found a yellow line or not, and see on which side of the robot it is on
     */
    int found_yellow_line_side();


    /**
     * @brief This function makes the robot spin
     * @param turn_deg The angle we want the robot to turn
     * @param first_time Checks if it is the first time entering a function
     * @return Checks if we have spun the desired angle
     */
    bool turn_left(double turn_deg, bool first_time);

    /**
     * @brief We calculate the current position of the robot
     */
    void calculate_actual_position();

    /**
     * @brief This function stops te robot and wait the desired time
     * @param seconds Time we want to wait
     * @param firstTime Checks if it is the first time entering a function
     * @return Checks if we have waited the desired time
     */
    bool wait_time(double seconds, bool firstTime);

    /**
     * @brief end_program This function is used when all the requirements have been met, and we can finish
     * @return Checks if the program has ended
     */
    bool end_program();
};
