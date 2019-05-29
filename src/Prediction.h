/*
 * Prediction.h
 *
 *  Created on: May 23, 2019
 *      Author: nora
 */

#ifndef SRC_PREDICTION_H_
#define SRC_PREDICTION_H_

#include <vector>

#include "Trajectory.h"

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

	static std::vector<Prediction> predict(
			double x, double y, double vx, double vy,
			double s, double d,
			std::vector<double> map_waypoints_x,
			std::vector<double> map_waypoints_y,
			std::vector<double> map_waypoints_s
			);
	PredictedBehavior get_behavior();

	double get_probability();

	const Trajectory& getTrajectory() const {
		return trajectory;
	}

private:

	PredictedBehavior behavior;

	double probability;

	// calculated only of probability > 0.1
	Trajectory trajectory;

};

#endif /* SRC_PREDICTION_H_ */

