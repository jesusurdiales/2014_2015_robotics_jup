/**
 * @file    main.cpp
 * @brief   A simple program for showing distance sensors values while the robot is moving.
 *
 * @author  Jesús Urdiales de la Parra
 * @date    2014-10
 */

#include "MyRobot.h"

/*
 * Main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
