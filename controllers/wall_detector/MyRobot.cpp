/**
 * @file    MyRobot.cpp
 * @brief   In this program the robot detects walls with frontal camera.
 *
 * @author  Jesús Urdiales de la Parra <nick@alumnos.uc3m.es>
 * @date    2014-11-13
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // We must to disable all devices on exit
    _forward_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int white_pixels = 0;

    // Get size of images for forward camera
    int image_width = _forward_camera->getWidth();
    int image_height = _forward_camera->getHeight();
    int image_pixels = image_width * image_height;

    cout << "Size of forward camera image: " << image_width << ", " <<  image_height << endl;

    while (step(_time_step) != -1)
    {
        // Get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage();

        // Calculate white pixels and the percentage of white in image.
        white_pixels = total_white_pixels(image_f, image_width, image_height);
        is_wall(image_pixels, white_pixels);

        // Set motor speeds necessary to turn around slowly
        _left_speed = 5;
        _right_speed = -5;
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

int MyRobot::total_white_pixels(const unsigned char *image, int im_width, int im_height)
{
    unsigned char green = 0, red = 0, blue = 0;
    int white_pixels = 0;

    // Count number of pixels that are white
    for (int x = 0; x < im_width; x++)
    {
        for (int y = 0; y < im_height; y++)
        {
            green = _forward_camera->imageGetGreen(image, im_width, x, y);
            red = _forward_camera->imageGetRed(image, im_width, x, y);
            blue = _forward_camera->imageGetBlue(image, im_width, x, y);

            // If all pixel components are equal and higger than 150, the robot considers that the pixel is white
            if ( (green == red) && (red == blue) && (blue > 150) )
                white_pixels++;
        }
    }
    return white_pixels;
}

void MyRobot::is_wall(int im_size, int total_white)
{
    double white_p = (total_white / (float) (im_size)) * 100;
    cout << "Percentage of white in forward camera image: " << white_p << endl;

    // If the percentage of white is bigger than 88 %, the robot considers that there is a wall in front.
    if (white_p >= 88)
    {
        cout << "I am in front of a wall" << endl;
    }
}

