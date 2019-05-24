/*
 * Behavior.h
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <vector>

using namespace std;

enum State {
	keep_lane, prepare_change_left, prepare_change_right, change_left, change_right
};

class Behavior {

public:
	Behavior();
	virtual ~Behavior();
	Behavior(
			const vector<double> map_waypoints_x,
			const vector<double> map_waypoints_y,
			const vector<double> map_waypoints_s);

	static const int TARGET_LANE = 1;

	State calculate_behavior(double car_s, int current_lane, double ego_speed, double t,
			vector<vector<double>> sensor_fusion);

	double get_closest_car_speed(double ego_s, int ego_lane, vector<vector<double>> sensor_fusion);

private:
	  vector<double> map_waypoints_x;
	  vector<double> map_waypoints_y;
	  vector<double> map_waypoints_s;
	  State current_state;

	  vector<State> get_next_states();

	  bool is_car_close(int lane, bool check_behind);

	  static bool is_in_same_lane(double d, int lane);

};

#endif /* BEHAVIOR_H_ */
