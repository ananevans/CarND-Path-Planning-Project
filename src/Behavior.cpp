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
#include "TrajectoryCostCalculator.h"


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
	//previous_lane = TARGET_LANE;
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
	vector<State> next_states = get_next_states();
	double min_cost = 2.0;
	Trajectory min_trajectory;
	double new_speed;
	State new_state;
	for (auto s = next_states.begin(); s != next_states.end(); ++s) {
		vector<Trajectory> trajectories = generate_trajectories(*s, car_x, car_y, car_s, car_d, car_speed, car_yaw,
				prev_path_x, prev_path_y, prev_end_s, prev_end_d);
		for ( auto ego_traj = trajectories.begin(); ego_traj < trajectories.end(); ++ego_traj) {
			double crt_cost =  TrajectoryCostCalculator::get_cost( *ego_traj, predictions );
			if ( crt_cost <= min_cost  ) {
				min_cost = crt_cost;
				min_trajectory = *ego_traj;
				new_speed = ego_traj->getTargetSpeed();
				new_state = *s;
			}
//			cout << "New min cost " << min_cost
//					<< " Trajectory cost = " << crt_cost << " "; ego_traj->debug_info();
		}
	}
	previous_speed = new_speed;
	this->current_state = new_state;
//	cout << "New speed " << previous_speed
//			<< " new state " << current_state
//			<< "\n";
	assert ( min_cost <= 1.0 );
	vector<vector<double>> xy(2);
	xy[0] = min_trajectory.getX();
	xy[1] = min_trajectory.getY();
	return xy;
}

vector<Trajectory> Behavior::generate_trajectories( State s,
			  double car_x, double car_y, double car_s, double car_d,
			  double car_speed, double car_yaw,
			  vector<double> prev_path_x, vector<double> prev_path_y, double prev_end_s, double prev_end_d) {
	TrajectoryGenerator gen(car_x, car_y, car_yaw, prev_path_x, prev_path_y,
			this->map_waypoints_x, this->map_waypoints_y, this->map_waypoints_s);
	int current_lane = Trajectory::get_lane(car_d);
	vector<Trajectory> result;
	// three possible trajectories: maintain speed, increase speed, decrease speed
	vector<double> delta_velocity = { 0.0, DELTA_VELOCITY_UP, DELTA_VELOCITY_DOWN, 2*DELTA_VELOCITY_DOWN, 3*DELTA_VELOCITY_DOWN};
	switch (s) {
	case keep_lane:
		for (int i=0; i<delta_velocity.size(); i++) {
			double target_velocity = this->previous_speed + delta_velocity[i];
			if (target_velocity > 0.0 ) {
				Trajectory trajectory = gen.build_trajectory( current_lane,
						current_lane,
						target_velocity );
				result.push_back(trajectory);
			}
		}
		return result;
	case prepare_change_left:
		if (current_lane > 0 ) {
			for (int i=0; i<delta_velocity.size(); i++) {
				double target_velocity = this->previous_speed + delta_velocity[i];
				if (target_velocity > 0.0 ) {
					Trajectory trajectory = gen.build_trajectory( current_lane,
							current_lane - 1,
							target_velocity );
					result.push_back(trajectory);
				}
			}
		}
		return result;
	case prepare_change_right: {
		if (current_lane < MAX_LANE) {
			for (int i=0; i<delta_velocity.size(); i++) {
				double target_velocity = this->previous_speed + delta_velocity[i];
				if (target_velocity > 0.0 ) {
					Trajectory trajectory = gen.build_trajectory( current_lane,
							current_lane+1,
							target_velocity );
					result.push_back(trajectory);
				}
			}
		}
		return result;
	}
	case change_left: {
		if (current_lane > 0) {
			Trajectory trajectory = gen.build_trajectory( current_lane - 1, current_lane - 1,
					this->previous_speed );
			result.push_back(trajectory);
		}
		return result;
	}
	case change_right: {
		if (current_lane < MAX_LANE) {
			Trajectory trajectory = gen.build_trajectory( current_lane + 1, current_lane + 1,
					this->previous_speed );
			result.push_back(trajectory);
		}
		return result;
	}
	default:
		assert(true);
		return {};
	}
}
