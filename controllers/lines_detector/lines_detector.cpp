/**
 * @file    lines_detector.cpp
 * @brief   With this controller the robot is able to detect yellow lines on the floor.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include "MyRobot.h"

/**
 * @brief Main program. It just executes robot run method.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
