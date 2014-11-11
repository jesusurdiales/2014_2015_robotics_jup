/**
 * @file    MyRobot.h
 * @brief   The robot avoids an obstacle only using odometry.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace webots;
using namespace std;

#define MAX_SPEED           50
#define ENCODER_RESOLUTION  5
#define WHEEL_RADIUS        0.0825
#define DISTANCE_B_WHEELS   0.32

class MyRobot : public DifferentialWheels
{
private:

    int _time_step;
    // Variables to store encoder values
    double _left_encoder, _right_encoder;
    // Variables to control wheels speed
    double _left_speed, _right_speed;

    enum Mode {
        CALCULATING_NEXT_POINT, TURNING_RIGHT, TURNING_LEFT, GO_AHEAD
    };
    Mode _robot_actual_mode;

    // Robot coordinates
    double _x;
    double _y;
    double _theta;

    // Calculate parameters between two points
    double _distance_to_objetive;
    double _angle_to_objetive;

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
     * @brief Function that controls robot mode.
     */
    void run();

    /**
     * @brief This function contains all target points coordinates.
     * @param next_point Next point to reach.
     * @return true if there are not more points to reach, false otherwise.
     */
    bool points_to_reach(int next_point);

    /**
     * @brief Keeps the robot in a straight line between two points.
     * @param pdistance Distance from last point.
     */
    void go_ahead (double *pdistance);

    /**
     * @brief Turns left the robot.
     */
    void turn_left ();

    /**
     * @brief Turns right the robot.
     */
    void turn_right ();

    /**
     * @brief This function uses odometry to calculate robot position.
     * @param pdistance Distance from last point.
     */
    void calculate_actual_position(double *pdistance);

    /**
     * @brief Function that calculates distance and angle to next point.
     * @param xx Coordinate x of next point.
     * @param yy Coordinate y of next point.
     */
    void set_next_point (double xx, double yy);

    /**
     * @brief Function that converts an absolute angle ( 0 <= angle < 2*PI) in a normalize angle.
     * @param angle Absolute angle.
     * @return The normalized angle in range between -PI, +PI.
     */
    double normalize_angle (double angle);

    /**
     * @brief Function that calculates absolute angle between actual and next point.
     * @param xi Coordinate x in actual position.
     * @param yi Coordinate y in actual position.
     * @param xf Coordinate x in next position.
     * @param yf Coordinate y in next position.
     * @return Absolute angle between actual and next point in radians.
     */
    double calculate_absolute_angle (double xi, double yi, double xf, double yf);
};
