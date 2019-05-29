/*
 * Trajectory.h
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#ifndef SRC_TRAJECTORYGENERATOR_H_
#define SRC_TRAJECTORYGENERATOR_H_

#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "Trajectory.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class TrajectoryGenerator {
public:

	TrajectoryGenerator(
			double car_x, double car_y, double car_yaw,
			vector<double> prev_x, vector<double> prev_y,
			vector<double> map_waypoints_x,
			vector<double> map_waypoints_y,
			vector<double> map_waypoints_s);

	virtual ~TrajectoryGenerator();

	Trajectory build_trajectory(
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

#endif /* SRC_TRAJECTORYGENERATOR_H_ */
