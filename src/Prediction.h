/*
 * Prediction.h
 *
 *  Created on: May 23, 2019
 *      Author: nora
 */

#ifndef SRC_PREDICTION_H_
#define SRC_PREDICTION_H_

#include <vector>

enum PredictedBehavior {
	predict_keep_lane, predict_change_left, predict_change_right
};

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

	static std::vector<Prediction> predict(int lane,
			double x, double y, double vx, double vy,
			double s, double d,
			std::vector<double> map_waypoints_x,
			std::vector<double> map_waypoints_y,
			std::vector<double> map_waypoints_s
			);
	PredictedBehavior get_behavior();

	double get_probability();

	std::vector<double> get_x();

	std::vector<double> get_y();

private:

	PredictedBehavior behavior;

	double probability;

	// x and y are calculated only of probability > 0.1
	std::vector<double> x;

	std::vector<double> y;

};

#endif /* SRC_PREDICTION_H_ */

