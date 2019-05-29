/*
 * Behavior.h
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <vector>

#include "Prediction.h"
#include "Trajectory.h"

using namespace std;

enum State {
	keep_lane, prepare_change_left, prepare_change_right, change_left, change_right
};

class Behavior {

public:
	virtual ~Behavior();
	Behavior(
			const vector<double> map_waypoints_x,
			const vector<double> map_waypoints_y,
			const vector<double> map_waypoints_s);

	static const int TARGET_LANE = 1;

	vector<vector<double>> calculate_behavior(
			double car_x, double car_y, double car_s, double car_d,
			double car_speed, double car_yaw,
			vector<double> prev_path_x, vector<double> prev_path_y, double prev_end_s, double prev_end_d,
			vector<vector<Prediction>> predictions);

private:
	  vector<double> map_waypoints_x;
	  vector<double> map_waypoints_y;
	  vector<double> map_waypoints_s;
	  State current_state;
	  double previous_speed;
	  int previous_lane;

	  vector<State> get_next_states();

	  vector<Trajectory> generate_trajectories( State s,
			  double car_x, double car_y, double car_s, double car_d,
			  double car_speed, double car_yaw,
			  vector<double> prev_path_x, vector<double> prev_path_y, double prev_end_s, double prev_end_d);
};

#endif /* BEHAVIOR_H_ */
