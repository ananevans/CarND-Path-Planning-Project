/*
 * Behavior.cpp
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#include "Behavior.h"
#include "Constants.h"
#include "Trajectory.h"
#include "TrajectoryGenerator.h"


#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

Behavior::~Behavior() {
	// TODO Auto-generated destructor stub
}

Behavior::Behavior(
		const vector<double> map_waypoints_x,
		const vector<double> map_waypoints_y,
		const vector<double> map_waypoints_s) {
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_s = map_waypoints_s;
	current_state = keep_lane;
	previous_speed = 0.0;
}

vector<State> Behavior::get_next_states() {
	switch (current_state) {
	case keep_lane: return {keep_lane, prepare_change_left, prepare_change_right};
	case prepare_change_left: return {keep_lane, prepare_change_left, change_left};
	case prepare_change_right: return {keep_lane, prepare_change_right, change_right};
	case change_left: return {change_left, keep_lane};
	case change_right: return {change_right, keep_lane};
	default: return {};
	}
}

vector<vector<double>> Behavior::calculate_behavior(
		double car_x, double car_y, double car_s, double car_d,
		double car_speed, double car_yaw,
		vector<double> prev_path_x, vector<double> prev_path_y, double prev_end_s, double prev_end_d,
		vector<vector<Prediction>> predictions) {
	// update state if needed
	if ( this->current_state == change_left || this->current_state == change_right) {
		if (this->previous_lane != Trajectory::get_lane(car_d)) {
			// lane was changed, update state to KL and lane
			this->current_state = keep_lane;
			this->previous_lane = Trajectory::get_lane(car_d);
		} // else the lane changing did not complete
	}
	vector<State> next_states = get_next_states();
	for (auto s = next_states.begin(); s != next_states.end(); ++s) {
		vector<Trajectory> trajectories = generate_trajectories(*s, car_x, car_y, car_s, car_d, car_speed, car_yaw,
				prev_path_x, prev_path_y, prev_end_s, prev_end_d);
	}
	// find trajectory of minimum cost
	return {}; // TODO
}

vector<Trajectory> Behavior::generate_trajectories( State s,
			  double car_x, double car_y, double car_s, double car_d,
			  double car_speed, double car_yaw,
			  vector<double> prev_path_x, vector<double> prev_path_y, double prev_end_s, double prev_end_d) {
	TrajectoryGenerator gen(car_x, car_y, car_yaw, prev_path_x, prev_path_y,
			this->map_waypoints_x, this->map_waypoints_y, this->map_waypoints_s);
	switch (s) {
	case keep_lane:
	case prepare_change_left:
	case prepare_change_right: {
		vector<Trajectory> result;
		// three possible trajectories: maintain speed, increase speed, decrease speed
		vector<double> delta_velocity = { 0.0, DELTA_VELOCITY_UP, DELTA_VELOCITY_DOWN};
		for (int i=0; i<delta_velocity.size(); i++) {
			double target_velocity = this->previous_speed + delta_velocity[i];
			Trajectory trajectory = gen.build_trajectory( this->previous_lane, target_velocity );
			result.push_back(trajectory);
		}
		return result;
	}
	case change_left: {
		Trajectory trajectory = gen.build_trajectory( this->previous_lane - 1, this->previous_speed );
		vector<Trajectory> result;
		result.push_back(trajectory);
		return result;
	}
	case change_right: {
		Trajectory trajectory = gen.build_trajectory( this->previous_lane + 1, this->previous_speed );
		vector<Trajectory> result;
		result.push_back(trajectory);
		return result;
	}
	default:
		assert(true);
		return {};
	}
}
// TODO

//double Behavior::cost( State next_state, double car_s, int current_lane, double ego_speed, double t,
//		vector<vector<double>> sensor_fusion  ) {
//	switch (next_state) {
//	case keep_lane:
//		return  SPEED_LIMIT_MPS - get_closest_car_speed( car_s, current_lane, sensor_fusion );
//	case prepare_change_left:
//		if ( current_lane == 0 ) {
//			// can't change left
//			return MAX_COST;
//		} else {
//
//		}
//		// left blocked, but there is a gap behind
//	case change_left: {
//		// cost 0 if matching speed and gap is next to me
//	}
//	}
//	return 0.0; // TODO
//}
//
//bool Behavior::is_in_same_lane( double d, int lane ) {
//	return (lane*LANE_WIDTH < d ) && (d<(lane+1)*LANE_WIDTH);
//}
//
//
//// returns value in mps
//double Behavior::get_closest_car_speed(double ego_s, int ego_lane, vector<vector<double>> sensor_fusion ) {
//	double target_speed = SPEED_LIMIT_MPS;
//	double min_dist = 100000;
//	for (int i=0; i < sensor_fusion.size(); i++ ) {
//		double s = sensor_fusion[i][5];
//		if (is_in_same_lane(  sensor_fusion[i][6], ego_lane ) ) {
//			if (ego_s < s && (s-ego_s) < min_dist) {
//				min_dist = s-ego_s;
//				target_speed = sqrt(sensor_fusion[i][3]*sensor_fusion[i][3] + sensor_fusion[i][4]*sensor_fusion[i][4]);
//			}
//		}
//	}
//	return target_speed;
//}
//
//
//bool Behavior::is_car_close(double ego_s, int lane, double ego_speed,
//		bool check_behind, vector<vector<double>> sensor_fusion) {
//
//	bool too_close = false;
//
//	for (int i=0; i < sensor_fusion.size(); i++ ) {
//		if (is_in_same_lane(  sensor_fusion[i][6], lane ) ) {
//			double speed = sqrt(sensor_fusion[i][3]*sensor_fusion[i][3] + sensor_fusion[i][4]*sensor_fusion[i][4]);
//			double s = sensor_fusion[i][5];
//			if ( ego_s < s) {
//				// car ahead
//				if ( s - ego_s < GAP ) {
//					// gap too small
//					if ( ego_speed >= speed ) {
//						// ego car is faster than the one ahead
//						too_close = true;
//					}
//				}
//			} else {
//				// car behind
//				if (check_behind) {
//					if ( ego_s - s < GAP  ) {
//						// car behind is within GAP
//						if (ego_s - s < GAP/2) {
//							// car too close
//							too_close = true;
//						} else {
//							if (ego_speed > speed) {
//								// ego car is faster
//							} else {
//								// car in less than half of gap and faster
//								too_close = true;
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//	return too_close;
//}

//int Behavior::calculate_behavior( ) {
//	if (is_car_close( ego_lane, false )) {
//		// car too close in ego lane
//		std::cout << "Car too close in ego lane with speed " << get_closest_car_speed() << "\n";
//		if ( ego_lane > 0 && !is_car_close(ego_lane - 1, true) ) {
//			std::cout << "Change lane\n";
//			return CHANGE_LANE_LEFT;
//		} else {
//			std::cout << "Car too close in left lane\n";
//			if (ego_lane < MAX_LANE && !is_car_close(ego_lane+1, true)) {
//				return CHANGE_LANE_RIGHT;
//			} else {
//				return DECREASE_SPEED;
//			}
//		}
//	} else {
//		// free space ahead
//		if ( ego_lane == TARGET_LANE ) {
//			return INCREASE_SPEED;
//		} else {
//			// try to return to middle
//			if (ego_lane < TARGET_LANE) {
//				std::cout << "Would like to change right\n";
//				// want to change lane right
//				if (ego_lane < MAX_LANE && !is_car_close(ego_lane + 1, true)) {
//					std::cout << "Change right\n";
//					return CHANGE_LANE_RIGHT;
//				} else {
//					std::cout << "Can't change right\n";
//					return INCREASE_SPEED;
//				}
//			} else {
//				std::cout << "Would like to change left\n";
//				// want to change lane left
//				if (ego_lane > 0 && !is_car_close(ego_lane - 1, true)) {
//					std::cout << "Change left\n";
//					return CHANGE_LANE_LEFT;
//				} else {
//					std::cout << "Can't change left\n";
//					return INCREASE_SPEED;
//				}
//			}
//		}
//	}
//}
