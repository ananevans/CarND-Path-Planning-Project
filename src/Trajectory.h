/*
 * Trajectory.h
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#include <iostream>
#include <string>
#include <vector>

#include "spline.h"

using namespace std;

class Trajectory {
public:
	Trajectory();

	Trajectory(
			double car_x, double car_y, double car_yaw,
			vector<double> prev_x, vector<double> prev_y,
			vector<double> map_waypoints_x,
			vector<double> map_waypoints_y,
			vector<double> map_waypoints_s);

	virtual ~Trajectory();

	vector<vector<double>> build_straight_trajectory();

	vector<vector<double>> build_trajectory(double ego_speed,
			vector <double> prev_x, vector<double> prev_y,
			int target_lane, double target_velocity);

private:
	  vector<double> map_waypoints_x;
	  vector<double> map_waypoints_y;
	  vector<double> map_waypoints_s;
	  vector<double> prev_x;
	  vector<double> prev_y;
	  double car_x;
	  double car_y;
	  double car_yaw;

	  static double get_d(int target_lane);

	  tk::spline get_spline(double x1, double y1, double x2, double y2, double yaw, int target_lane);

};

#endif /* SRC_TRAJECTORY_H_ */