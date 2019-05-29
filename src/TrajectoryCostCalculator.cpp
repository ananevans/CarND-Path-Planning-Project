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
#include <assert.h>


///////////////////
// Feasibility
///////////////////
/**
 * Return 1 if the trajectory overlaps with predicted trajectories, 0 otherwise.
 */
double collision_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	assert( ego_trajectory.getD().size() == NO_POINTS );
	assert( ego_trajectory.getS().size() == NO_POINTS );
	for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
		for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
			assert( prediction->getTrajectory().getD().size() == NO_POINTS );
			assert( prediction->getTrajectory().getS().size() == NO_POINTS );
			for ( int i = 0; i < NO_POINTS; i++ ) {
				// check s-separation and d-separation
				if ( abs( prediction->getTrajectory().getD()[i] -  ego_trajectory.getD()[i]) < 2.2 ) {
					// cars less than one car width apart
					if ( abs( prediction->getTrajectory().getS()[i] -  ego_trajectory.getS()[i]) < 4.5 ) {
						//cars less than one car length apart
						return 1.0;
					}
				}
			}
		}
	}
	return 0.0;
}


///////////////////
// Safety
///////////////////
double lane_speed_match(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	// find car ahead and behind in target lane
	double closest_speed = SPEED_LIMIT_MPS + 1.0;
	double min_s_dist = 100;
	assert(ego_trajectory.getS().size() > 0);
	double ego_s = ego_trajectory.getS().back();
	for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
		for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
			assert(prediction->getTrajectory().getS().size() > 0);
			if ( prediction->getTrajectory().getTargetLane()  ==  ego_trajectory.getTargetLane()) {
				double s = prediction->getTrajectory().getS().back();
				if ( abs(s-ego_s) < min_s_dist ) {
					min_s_dist = abs(s-ego_s);
					closest_speed = prediction->getTrajectory().getTargetSpeed();
				}
			}
		}
	}
	if ( ego_trajectory.getTargetSpeed() > 0.0 && closest_speed < SPEED_LIMIT_MPS ) {
		return abs( ego_trajectory.getTargetSpeed() - closest_speed ) / ego_trajectory.getTargetSpeed();
	} else {
		return 0.0;
	}
}

double buffer_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	assert( ego_trajectory.getD().size() == NO_POINTS );
	assert( ego_trajectory.getS().size() == NO_POINTS );
	for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
		for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
			assert( prediction->getTrajectory().getD().size() == NO_POINTS );
			assert( prediction->getTrajectory().getS().size() == NO_POINTS );
			for ( int i = 0; i < NO_POINTS; i++ ) {
				if ( Trajectory::get_lane( ego_trajectory.getD()[i] ) ==
						Trajectory::get_lane( prediction->getTrajectory().getD()[i] ) ) {
					// cars in the same lane
					if ( abs( prediction->getTrajectory().getS()[i] -  ego_trajectory.getS()[i]) < 2 * ego_trajectory.getTargetSpeed() ) {
						//cars less than 2 seconds apart
						assert( ego_trajectory.getTargetSpeed() > 0.0 );
						return abs(prediction->getTrajectory().getS()[i] -  ego_trajectory.getS()[i])/2 * ego_trajectory.getTargetSpeed();
					}
				}
			}
		}
	}
	return 0.0;
}




///////////////////
// Legality
///////////////////

double speed_limit_violation(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	if (ego_trajectory.getTargetSpeed() >= SPEED_LIMIT_MPS) {
		return 1.0;
	} else {
		return 0.0;
	}
}


///////////////////
// Comfort
///////////////////

///////////////////
// Efficiency
///////////////////
double ego_speed_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	return abs(SPEED_LIMIT_MPS - ego_trajectory.getTargetSpeed()) / SPEED_LIMIT_MPS;
}

double target_lane_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	if (ego_trajectory.getTargetLane() != TARGET_LANE ) {
		return 0.0;
	} else {
		return 1.0;
	}
}



double TrajectoryCostCalculator::WEIGHTS[NO_COST_FN] = {
		1.0};
CostFunctionPtr TrajectoryCostCalculator::COST_FUNCTIONS[NO_COST_FN] = {
		collision_cost
//		,speed_limit_violation,
//		target_lane_cost
};

double TrajectoryCostCalculator::get_cost( Trajectory ego_trajectory, vector<vector<Prediction>> predictions ) {
	double result = 0.0;
	for (int i = 0; i < NO_COST_FN; i++) {
		result += TrajectoryCostCalculator::WEIGHTS[i] * (TrajectoryCostCalculator::COST_FUNCTIONS[i](ego_trajectory, predictions));
	}
	return result/NO_COST_FN;
}

