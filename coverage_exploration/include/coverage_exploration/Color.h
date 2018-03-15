#ifndef COLOR_H
#define COLOR_H

#include<ros/ros.h>
#include<std_msgs/ColorRGBA.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace ros;

class Color
{
public:
    Color() {

    }

	Color(double r, double g, double b, double a = 1.) {
		color.r = r;
		color.g = g;
		color.b = b;
		color.a = a;
	}

	std_msgs::ColorRGBA getColor() {
		return color;
	}

	/**
 * @function randomColor
 * @brief Produces a random color given a random object
 */
	static cv::Scalar randomColor( cv::RNG& rng )
	{
		int icolor = (unsigned) rng;
		return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
	}

protected:
	std_msgs::ColorRGBA color;
};

#endif

