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
#include <iostream>


/**
 * Slowest car ahead within 45m in the given lane.
 * TARGET_SPEED if no cars ahead.
 */
double get_lane_speed(vector<vector<Prediction>> predictions, int lane, double s ) {
	double min_speed = TARGET_SPEED;
	for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
		for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
			double vehicle_s = prediction->getTrajectory().getS().back();
			double vehicle_d = prediction->getTrajectory().getD().back();
			if ( Trajectory::get_lane( vehicle_d ) == lane ) {
				// same lane
				if ( vehicle_s < s + 45.0 && s < vehicle_s) {
					if ( prediction->getTrajectory().getTargetSpeed() < min_speed ) {
						min_speed = prediction->getTrajectory().getTargetSpeed();
					}
				}
			}
		}
	}
	return min_speed;
}

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
				if ( fabs( prediction->getTrajectory().getD()[i] -  ego_trajectory.getD()[i]) < 2.2 ) {
					// cars less than one car width apart
					if ( fabs( prediction->getTrajectory().getS()[i] -  ego_trajectory.getS()[i]) < 5 ) {
						//cars less than one car length apart
						return 1.0;
					}
				}
			}
		}
	}
	return 0.0;
}

double road_limits(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	assert( ego_trajectory.getD().size() == NO_POINTS );
	for ( int i = 0; i < NO_POINTS; i++ ) {
		if ( ego_trajectory.getD()[i] <= 1.1 ||
				ego_trajectory.getD()[i] >= LANE_WIDTH * (MAX_LANE + 1) - 1.1 ) {
			return 1.0;
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
// Safety
///////////////////
double lane_speed_match(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	double lane_speed = get_lane_speed( predictions, ego_trajectory.getTargetLane(), ego_trajectory.getS().back() );
	lane_speed = min(TARGET_SPEED, lane_speed);
	return fabs(lane_speed - ego_trajectory.getTargetSpeed())/TARGET_SPEED;
}

double buffer_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	assert( ego_trajectory.getD().size() == NO_POINTS );
	assert( ego_trajectory.getS().size() == NO_POINTS );
	double cost = 0.0;
	for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
		for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
			assert( prediction->getTrajectory().getD().size() == NO_POINTS );
			assert( prediction->getTrajectory().getS().size() == NO_POINTS );
			double ego_s = ego_trajectory.getS().back();
			double ego_d = ego_trajectory.getD().back();
			double vehicle_s = prediction->getTrajectory().getS().back();
			double vehicle_d = prediction->getTrajectory().getD().back();

			if ( Trajectory::get_lane( ego_d ) == Trajectory::get_lane( vehicle_d ) ) {
				// cars in the same lane
				if ( ego_s  <  vehicle_s ) {
					// car ahead
					if ( vehicle_s - ego_s < 2 * ego_trajectory.getTargetSpeed() ) {
						// within 2s
						double new_cost = (2 * ego_trajectory.getTargetSpeed() - (vehicle_s - ego_s)) /
								(2 * ego_trajectory.getTargetSpeed());
						if ( new_cost > cost ) {
							cost = new_cost;
						}
					}
				}
			}
		}
	}
	return cost;
}

double gap_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	if ( ego_trajectory.getTargetLane() != ego_trajectory.getFinalLane() ) {
		//prepare lane change or lane change
		for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
			for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
				assert( prediction->getTrajectory().getD().size() == NO_POINTS );
				assert( prediction->getTrajectory().getS().size() == NO_POINTS );
				double ego_s = ego_trajectory.getS().back();
				double vehicle_s = prediction->getTrajectory().getS().back();
				double vehicle_d = prediction->getTrajectory().getD().back();
				if ( Trajectory::get_lane( vehicle_d ) == ego_trajectory.getFinalLane() ) {
					// vehicle in final lane
					double speed = max( ego_trajectory.getTargetSpeed(), prediction->getTrajectory().getTargetSpeed() );
					if ( fabs(vehicle_s - ego_s) < 2 * speed ) {
						return (2 * speed - fabs(vehicle_s - ego_s))/(2 * speed) ;
					}
				}
			}
		}
	}
	return 0.0;
}


double lane_crossing_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	int lane0 = 0;
	int lane1 = 0;
	vector<double> d = ego_trajectory.getD();
	for (int i = 0; i < d.size(); i++) {
		if ( fabs( d[i] - LANE_WIDTH ) < 0.5 ) {
			lane0 += 1;
		} else {
			if ( fabs( d[i] - 2 * LANE_WIDTH ) < 0.5 ) {
				lane1 += 1;
			}
		}
	}
	return 1 - exp( -max(lane0,lane1) );
}

///////////////////
// Comfort
///////////////////

///////////////////
// Efficiency
///////////////////
//double ego_speed_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
//	return abs(TARGET_SPEED - ego_trajectory.getTargetSpeed()) / TARGET_SPEED;
//}

double target_lane_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	if (ego_trajectory.getFinalLane() != TARGET_LANE ) {
		return 1.0;
	} else {
		return 0.0;
	}
}


double inefficinecy_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	double ego_s = ego_trajectory.getS().back();
	double target_lane_speed = min( TARGET_SPEED,
			get_lane_speed(predictions, ego_trajectory.getTargetLane(), ego_s));
	double final_lane_speed = min( TARGET_SPEED,
			get_lane_speed(predictions, ego_trajectory.getFinalLane(), ego_s));
//	cout << "inefficinecy_cost target_lane_speed=" << target_lane_speed << "\n";
//	cout << "inefficinecy_cost final_lane_speed=" << final_lane_speed << "\n";
//	cout << "inefficinecy_cost TARGET_SPEED=" << TARGET_SPEED << "\n";
	double cost = (2.0*TARGET_SPEED - target_lane_speed - final_lane_speed)/(2.0*TARGET_SPEED);
	return cost;
}



double zero_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	return 0.0;
}

double TrajectoryCostCalculator::WEIGHTS[NO_COST_FN] = {
		  10000
		, 10000
		, 10000
		, 1
		//, 50
		, 80
		, 100
		, 100
		, 1000
		, 80
		, 0
};
CostFunctionPtr TrajectoryCostCalculator::COST_FUNCTIONS[NO_COST_FN] = {
		  collision_cost
		, road_limits
		, speed_limit_violation
		, target_lane_cost
		, inefficinecy_cost
		, buffer_cost
		, lane_speed_match
		, gap_cost
		, lane_crossing_cost
		, zero_cost
};

string names[NO_COST_FN] = {
		  "collision_cost"
		, "road_limits"
		, "speed_limit_violation"
		, "target_lane_cost"
		, "inefficinecy_cost"
		, "buffer_cost"
		, "lane_speed_match"
		, "gap_cost"
		, "lane_crossing_cost"
		, "zero_cost"
};

double TrajectoryCostCalculator::get_cost( Trajectory ego_trajectory, vector<vector<Prediction>> predictions ) {
	double result = 0.0;
	double sum = 0.0;
	for (int i = 0; i < NO_COST_FN; i++) {
		double cost = (TrajectoryCostCalculator::COST_FUNCTIONS[i](ego_trajectory, predictions));
		result += TrajectoryCostCalculator::WEIGHTS[i] * cost;
		sum += TrajectoryCostCalculator::WEIGHTS[i];
		//cout << "Weighted cost  function " << names[i] << ": " << WEIGHTS[i] * cost << "\n";
	}
	return result/sum;
}

