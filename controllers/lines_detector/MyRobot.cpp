/**
 * @file    MyRobot.cpp
 * @brief   With this controller the robot is able to detect yellow lines on the floor.
 *
 * @author  Jesús Urdiales de la Parra <100291428@alumnos.uc3m.es>
 * @date    2014-11-08
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int yellow_pixels = 0;

    // Get size of images for spherical camera
    int image_width = _spherical_camera->getWidth();
    int image_height = _spherical_camera->getHeight();
    int image_pixels = image_width * image_height;

    cout << "Size of spherical camera image: " << image_width << ", " << image_height << endl;

    while (step(_time_step) != -1)
    {
        // Get current image from spherical camera
        const unsigned char *image_s = _spherical_camera->getImage();

        // Calculate yellow pixels and the percentage of yellow in image.
        yellow_pixels = total_yellow_pixels(image_s, image_width, image_height);
        is_line(image_pixels, yellow_pixels);

        // Set motor speeds necessary to turn around slowly
        _left_speed = 5;
        _right_speed = -5;
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

int MyRobot::total_yellow_pixels(const unsigned char *image, int im_width, int im_height)
{
    unsigned char green = 0, red = 0, blue = 0;
    int yellow_pixels = 0;

    // Count number of pixels that are yellow
    for (int x = 0; x < im_width; x++)
    {
        for (int y = 0; y < im_height; y++)
        {
            green = _spherical_camera->imageGetGreen(image, im_width, x, y);
            red = _spherical_camera->imageGetRed(image, im_width, x, y);
            blue = _spherical_camera->imageGetBlue(image, im_width, x, y);

            // Test if actual pixel is yellow
            if ((green > 250) && (red > 250) && (blue < 10))
                yellow_pixels++;
        }
    }
    return yellow_pixels;
}

void MyRobot::is_line(int im_size, int total_yellow)
{
    double yellow_p = (total_yellow / (float) (im_size)) * 100;
    cout << "Percentage of yellow in spherical camera image: " << yellow_p << endl;

    // If the percentage of yellow is bigger than 1 %, the robot considers that there is a yellow line
    if (yellow_p >= 1)
    {
        cout << "Yellow line detected!" << endl;
    }
}

