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

////////////////////
// Helpers
///////////////////
vector<double> get_closest_speeds_ahead( Trajectory ego_trajectory, vector<vector<Prediction>> predictions ) {
	double closest_speed_target = SPEED_LIMIT_MPS;
	double closest_speed_final = SPEED_LIMIT_MPS;
	double min_s_dist_target = 50.0;
	double min_s_dist_final = 50.0;
	assert(ego_trajectory.getS().size() > 0);
	double ego_s = ego_trajectory.getS().back();
	for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
		for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
			assert(prediction->getTrajectory().getS().size() > 0);
			double s = prediction->getTrajectory().getS().back();
			if ( s < ego_s ) {
				if ( prediction->getTrajectory().getTargetLane()  ==  ego_trajectory.getTargetLane()) {
					if (  ego_s - s < min_s_dist_target ) {
						min_s_dist_target = ego_s - s;
						closest_speed_target = prediction->getTrajectory().getTargetSpeed();
					}
				}
				if ( prediction->getTrajectory().getTargetLane() == ego_trajectory.getFinalLane() ) {
					if ( ego_s - s < min_s_dist_final ) {
						min_s_dist_final = abs(s-ego_s);
						closest_speed_final = prediction->getTrajectory().getTargetSpeed();
					}
				}
			}
		}
	}
	vector<double> result;
	result.push_back(closest_speed_target);
	result.push_back(closest_speed_final);
	return result;
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
				if ( abs( prediction->getTrajectory().getD()[i] -  ego_trajectory.getD()[i]) < 2.2 ) {
					// cars less than one car width apart
					if ( abs( prediction->getTrajectory().getS()[i] -  ego_trajectory.getS()[i]) < 5 ) {
						//cars less than one car length apart
//						cout << "collision_cost::Collision detected for trajectory "; ego_trajectory.debug_info();
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

//			cout << "road_limits::Violates road limits " << ego_trajectory.getD()[i] << "\n";

			return 1.0;
		}
	}
	return 0.0;
}


///////////////////
// Safety
///////////////////
double lane_speed_match(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	double closest_speed = 100*SPEED_LIMIT_MPS;
	double min_s_dist = 50.0;
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
	if ( ego_trajectory.getTargetSpeed() > 0.0 && min_s_dist < 50.0 ) {
//		cout << "lane_speed_match::Closest speed " << closest_speed << " s-distance " << min_s_dist << "\n";
//		cout << "lane_speed_match::Ego target speed " << ego_trajectory.getTargetSpeed()<< "\n";;
//		cout << "lane_speed_match::lane_speed_match cost" << abs( ego_trajectory.getTargetSpeed() - closest_speed ) / SPEED_LIMIT_MPS<< "\n";;
		return abs( ego_trajectory.getTargetSpeed() - closest_speed ) / SPEED_LIMIT_MPS;
	} else {
//		cout << "lane_speed_match::No car ahead within 100m\n";
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
					double ego_s = ego_trajectory.getS()[i];
					double other_s = prediction->getTrajectory().getS()[i];
					// cars in the same lane
					if ( ego_s  <  other_s ) {
						// car ahead
						if ((other_s - ego_s) <  ego_trajectory.getTargetSpeed()) {
							return 1.0;
						} else {
							if ((other_s - ego_s) <  2*ego_trajectory.getTargetSpeed()) {
								return min(0.5, (other_s - ego_s)/(2*ego_trajectory.getTargetSpeed()) );
							}
						}
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

	return abs(TARGET_SPEED - ego_trajectory.getTargetSpeed()) / TARGET_SPEED;
}

double target_lane_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	if (ego_trajectory.getFinalLane() != TARGET_LANE ) {
		return 1.0;
	} else {
		return 0.0;
	}
}

double lane_changing_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	vector<double> speeds = get_closest_speeds_ahead(ego_trajectory, predictions);
	double target_lane_speed = min( speeds[0], TARGET_SPEED );
	double final_lane_speed = min( speeds[1], TARGET_SPEED);
	double avg = (target_lane_speed+final_lane_speed)/2;
	double cost = (TARGET_SPEED - avg)/TARGET_SPEED;
//	double cost = (2*TARGET_SPEED - target_lane_speed - final_lane_speed)/TARGET_SPEED;
	return cost;
}

double gap_size(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
	if ( ego_trajectory.getTargetLane() != ego_trajectory.getFinalLane() ) {
		// want a gap alongside
		double ego_s = ego_trajectory.getS()[0];
		double ahead_s = ego_s + 30;
		double behind_s = ego_s - 30;
		for (auto vehicle_predictions = predictions.begin(); vehicle_predictions < predictions.end(); ++vehicle_predictions) {
			for (auto prediction = vehicle_predictions->begin(); prediction < vehicle_predictions->end(); ++prediction) {
				if ( ego_trajectory.getFinalLane() == prediction->getTrajectory().getTargetLane() ) {
					double vehicle_s = prediction->getTrajectory().getS()[0];
					if ( vehicle_s < ego_s ) {
						// vehicle ahead
						if ( vehicle_s < ahead_s ) {
							// closer to the eqo
							ahead_s = vehicle_s;
						}
					} else {
						if ( behind_s < vehicle_s ) {
							behind_s = vehicle_s;
						}
					}
				}
			}
		}
		return (60.0 - (ahead_s - behind_s) )/ 60.0;
	} else {
		return 0.0;
	}
}

double TrajectoryCostCalculator::WEIGHTS[NO_COST_FN] = {
		  1.0
		, 1.0
		, 0.0003
		, 0.000001
		, 0.009
		, 0.008
		, 0.009
		, 0.0001
		, 0.0001
};
CostFunctionPtr TrajectoryCostCalculator::COST_FUNCTIONS[NO_COST_FN] = {
		  collision_cost
		, road_limits
		, ego_speed_cost
		, target_lane_cost
		, lane_speed_match
		, buffer_cost
		, speed_limit_violation
		, lane_changing_cost

		, gap_size
};

string names[NO_COST_FN] = {
		  "collision_cost"
		, "road_limits"
		, "ego_speed_cost"
		, "target_lane_cost"
		, "lane_speed_match"
		, "buffer_cost"
		, "speed_limit_violation"
		, "lane_changing_cost"

		, "gap_size"

};

double TrajectoryCostCalculator::get_cost( Trajectory ego_trajectory, vector<vector<Prediction>> predictions ) {
	double result = 0.0;
	for (int i = 0; i < NO_COST_FN; i++) {
		double cost = (TrajectoryCostCalculator::COST_FUNCTIONS[i](ego_trajectory, predictions));
		result += TrajectoryCostCalculator::WEIGHTS[i] * cost;
		cout << "Weighted cost  function " << names[i] << ": " << WEIGHTS[i] * cost << "\n";
	}
	return result/NO_COST_FN;
}



//double speed_jerk_acceleration_cost(Trajectory ego_trajectory, vector<vector<Prediction>> predictions) {
//	vector<double> x = ego_trajectory.getX();
//	vector<double> y = ego_trajectory.getY();
//	// speed check
//	{
//		vector<double> vx, vy, ax, ay;
//		for ( int i = 0; i < x.size() - 1; i++ ) {
//			vx.push_back( (x[i+1] - x[i]) / DELTA_T  );
//			vy.push_back( (y[i+1] - y[i]) / DELTA_T  );
//			double speed = sqrt(vx[i]*vx[i] + vy[i]*vy[i]);
//			if ( speed >= SPEED_LIMIT_MPS) {
//				cout << "Speed violation position " << i
//						<< " value " << speed << "\n";
//				return 1.0;
//			}
//		}
//	}
//	vector<double> s = ego_trajectory.getS();
//	vector<double> d = ego_trajectory.getD();
//	// speed check
//	vector<double> vs, vd, as, ad;
//	for ( int i = 0; i < x.size() - 1; i++ ) {
//		vs.push_back( (s[i+1] - s[i]) / DELTA_T  );
//		vd.push_back( (d[i+1] - d[i]) / DELTA_T  );
//	}
//	// acceleration check
//	for (int i = 0; i < vs.size() - 1; i++ ) {
//		as.push_back( (vs[i+1] - vs[i])/DELTA_T );
//		ad.push_back( (vd[i+1] - vd[i])/DELTA_T);
//		double acceleration = sqrt(as[i]*as[i] + ad[i]*ad[i]);
//		if ( acceleration >= 10) {
//			cout << "Acceleration violation position " << i
//					<< " value " << acceleration << "\n";
//			return 1.0;
//		}
//	}
//	// jerk check
//	for (int i = 0; i < as.size() - 1; i++) {
//		double jerk = (as[i+1] - as[i])/DELTA_T;
//		//double js = (as[i+1] - as[i])/DELTA_T;
//		//double jd = (ad[i+1] - ad[i])/DELTA_T;
//		//double jerk = sqrt(js*js + jd*jd);
//		if ( jerk >= 10 ) {
//			cout << "Jerk violation position " << i
//				<< " value " << jerk << "\n";
//			return 1.0;
//		}
//	}
//	return 0.0;
//}
