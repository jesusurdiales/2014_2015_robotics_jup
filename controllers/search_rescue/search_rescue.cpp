/**
 * @file    search_rescue.cpp
 * @brief   Full final proyect that makes a robot save 2 people from a hazard
 *
 * @author  Jesús Urdiales de la Parra          <100291428@alumnos.uc3m.es>
 * @author  Guillermo Mendiguchia Martinez      <100284346@alumnos.uc3m.es>
 * @date    2014-12-05
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
