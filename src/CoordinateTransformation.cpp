/*
 * CoordinateTransformation.cpp
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#include <math.h>

#include "CoordinateTransformation.h"

CoordinateTransformation::CoordinateTransformation(double xo, double yo, double theta) {
	this->xo = xo;
	this->yo = yo;
	this->theta = -theta;
}

CoordinateTransformation::~CoordinateTransformation() {
}

double CoordinateTransformation::to_x( double x, double y ) {
	return (x-xo)*cos(theta) - (y-yo) * sin(theta);
}

double CoordinateTransformation::to_y( double x, double y ) {
	return (x-xo)*sin(theta) + (y-yo)*cos(theta);
}

double CoordinateTransformation::from_x( double x_prime, double y_prime ){
	return xo + x_prime*cos(theta) + y_prime*sin(theta);
}

double CoordinateTransformation::from_y( double x_prime, double y_prime ) {
	return yo - x_prime*sin(theta) + y_prime*cos(theta);
}



