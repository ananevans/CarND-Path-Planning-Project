/*
 * CoordinateTransformation.h
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#ifndef SRC_COORDINATETRANSFORMATION_H_
#define SRC_COORDINATETRANSFORMATION_H_

class CoordinateTransformation {
public:
	CoordinateTransformation(double xo, double yo, double theta);
	virtual ~CoordinateTransformation();

	double to_x( double x, double y );
	double to_y( double x, double y );

	double from_x( double x_prime, double y_prime );
	double from_y( double x_prime, double y_prime );

private:
	double xo, yo, theta;
};

#endif /* SRC_COORDINATETRANSFORMATION_H_ */
