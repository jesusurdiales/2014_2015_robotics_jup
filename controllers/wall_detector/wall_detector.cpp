/**
 * @file    wall_detector.cpp
 * @brief   In this program the robot detects walls with frontal camera.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-13
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
