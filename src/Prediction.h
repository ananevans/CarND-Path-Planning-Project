/*
 * Prediction.h
 *
 *  Created on: May 23, 2019
 *      Author: nora
 */

#ifndef SRC_PREDICTION_H_
#define SRC_PREDICTION_H_

#include <vector>

/**
 * Simple prediction based on the d coordinate.
 */
class Prediction {

public:
	Prediction();
	virtual ~Prediction();

	static const int KEEP_LANE_INDEX = 0;

	static const int CHANGE_LEFT_INDEX = 1;

	static const int CHANGE_RIGHT_INDEX = 2;

	static std::vector<double> predict( double d, int lane );

};

#endif /* SRC_PREDICTION_H_ */
