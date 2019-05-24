/*
 * Prediction.cpp
 *
 *  Created on: May 23, 2019
 *      Author: nora
 */
#include <vector>
#include <random>
#include <assert.h>

#include "Prediction.h"

#include "Constants.h"

Prediction::Prediction() {
	// TODO Auto-generated constructor stub

}

Prediction::~Prediction() {
	// TODO Auto-generated destructor stub
}


std::vector<double> Prediction::predict(double d, int lane) {
	std::vector<double> result(3, 0.0);
	double left = lane * LANE_WIDTH;
	double right = (lane+1) * LANE_WIDTH;
	double middle = (lane + 0.5) * LANE_WIDTH;

	double std_dev = LANE_WIDTH / 4;

	std::default_random_engine generator;
	std::normal_distribution<double> left_distribution(left, std_dev);
	std::normal_distribution<double> middle_distribution(middle, std_dev);
	std::normal_distribution<double> right_distribution(right, std_dev);

	if ( lane != 0 ) {
		result[Prediction::CHANGE_LEFT_INDEX] = left_distribution(generator);
	}
	result[Prediction::KEEP_LANE_INDEX] = middle_distribution(generator);
	if ( lane != MAX_LANE ) {
		result[Prediction::CHANGE_RIGHT_INDEX] = right_distribution(generator);
	}

	double sum = result[0] + result[1] + result[2];
	assert(sum > 0.0);
	for ( int i=0 ; i < result.size(); i++) {
		result[i] = result[i]/sum;
	}
	return result;
}
