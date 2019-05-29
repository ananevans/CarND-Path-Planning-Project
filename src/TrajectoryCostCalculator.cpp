/*
 * TrajectoryCostCalculator.cpp
 *
 *  Created on: May 28, 2019
 *      Author: nora
 */

#include "TrajectoryCostCalculator.h"

#include "Constants.h"
#include "helpers.h"

#include <math.h>

/**
 * Return 1 if the trajectory overlaps with predicted trajectories, 0 otherwise.
 */
double collision_cost(Trajectory ego_trajectory, vector<Prediction> predictions) {
	for (auto prediction = predictions.begin(); prediction < predictions.end(); ++prediction) {
		for ( int i = 0; i < NO_POINTS; i++ ) {
			// check s-separation and d-separation
			if ( abs( prediction->getTrajectory().getS()[i] - ego_trajectory.getS()[i] ) < 5.0 ) {
				return 1.0;
			}
			if ( abs( prediction->getTrajectory().getS()[i] - ego_trajectory.getS()[i] ) < 3.0 ) {
				return 1.0;
			}
		}
	}
	return 0.0;
}

double speed_limit_violation(Trajectory ego_trajectory, vector<Prediction> predictions) {
	if (ego_trajectory.getTargetSpeed() >= SPEED_LIMIT_MPS) {
		return 1.0;
	} else {
		return 0.0;
	}
}

double target_lane_cost(Trajectory ego_trajectory, vector<Prediction> predictions) {
	if (ego_trajectory.getTargetLane() != TARGET_LANE ) {
		return 0.0;
	} else {
		return 1.0;
	}
}

// TODO
//double ego_speed_cost() {
//	return
//}


static double WEIGHTS[NO_COST_FN] = {1.0, 0.1, 1.0};
static CostFunctionPtr COST_FUNCTIONS[NO_COST_FN] = {collision_cost,target_lane_cost, speed_limit_violation};

double TrajectoryCostCalculator::get_cost( Trajectory ego_trajectory, vector<Prediction> predictions ) {
	double result = 0.0;
	for (int i = 0; i < NO_COST_FN; i++) {
		result += WEIGHTS[i] * (COST_FUNCTIONS[i](ego_trajectory, predictions));
	}
	return result/NO_COST_FN;
}

