/**
 * @file    MyRobot.h
 * @brief   In this program the robot detects walls with frontal camera.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-13
 */

#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 2
#define DISTANCE_LIMIT      100
#define MAX_SPEED           100

class MyRobot : public DifferentialWheels
{
private:

    Camera * _forward_camera;

    int _time_step;
    double _left_speed, _right_speed;

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
     * @brief Function that acquires images from frontal camera and determines when the robot is in front of a wall.
     */
    void run();

    /**
     * @brief Function that calculates how many white pixels has an image.
     * @param image Image captured by frontal camera.
     * @param im_width Image width.
     * @param im_height Image height.
     * @return Total white pixels captured by frontal camera.
     */
    int total_white_pixels(const unsigned char *image, int im_width, int im_height);

    /**
     * @brief Function that determines if the robot is in front of a wall. Also warns of this.
     * @param im_size Total number of pixels.
     * @param total_white Total number of white pixels.
     */
    void is_wall(int im_size, int total_white);
};
