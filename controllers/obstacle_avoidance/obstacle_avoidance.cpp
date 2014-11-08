/**
 * @file    obstacle_avoidance.cpp
 * @brief   A program that shows how the robot avoids obstacles in an unknown enviroment.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include "MyRobot.h"

/**
 * @brief Main program. In this case main program just creates a robot instance, calls run method and destroys the robot.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
