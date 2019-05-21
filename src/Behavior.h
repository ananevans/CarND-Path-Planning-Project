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

class Behavior {

public:
	Behavior();
	virtual ~Behavior();
	Behavior(
			double car_s, int current_lane, double ego_speed,
			vector<vector<double>> sensor_fusion,
			const vector<double> map_waypoints_x,
			const vector<double> map_waypoints_y,
			const vector<double> map_waypoints_s);

	static const int INCREASE_SPEED = 0;

	static const int DECREASE_SPEED = 1;

	static const int CHANGE_LANE_LEFT = 2;

	static const int CHANGE_LANE_RIGHT = 3;

	static const int TARGET_LANE = 1;

	int calculate_behavior();

	double get_closest_car_speed();

private:
	  vector<double> map_waypoints_x;
	  vector<double> map_waypoints_y;
	  vector<double> map_waypoints_s;
	  double ego_s;
	  int ego_lane;
	  double ego_speed;
	  vector<vector<double>> sensor_fusion;

	  bool is_car_close(int lane, bool check_behind);

	  static bool is_in_same_lane(double d, int lane);

};

#endif /* BEHAVIOR_H_ */
