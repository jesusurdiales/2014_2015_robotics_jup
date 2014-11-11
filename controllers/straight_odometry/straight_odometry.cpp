/**
 * @file    straight_odometry.cpp
 * @brief   Program that shows the robot using odometry to move across the map.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include "MyRobot.h"

/**
 * @brief Main program.It just creates a robot instance, launches the run method and destroys the robot.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
