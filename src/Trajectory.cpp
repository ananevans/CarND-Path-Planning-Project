/*
 * Trajectory.cpp
 *
 *  Created on: May 17, 2019
 *      Author: nora
 */

#include "Trajectory.h"
#include "Constants.h"

#include "helpers.h"

#include "CoordinateTransformation.h"

#include <iostream>
#include <vector>

#define DISTANCE_AHEAD 45.0
#define WAYPOINTS_DISTANCE 30.0

using namespace std;


Trajectory::Trajectory() {
	// TODO Auto-generated constructor stub

}

Trajectory::~Trajectory() {
	// TODO Auto-generated destructor stub
}


Trajectory::Trajectory(
		double car_x, double car_y, double car_yaw,
		vector<double> prev_x, vector<double> prev_y,
		const vector<double> map_waypoints_x,
		const vector<double> map_waypoints_y,
		const vector<double> map_waypoints_s) {
	this->car_x = car_x;
	this->car_y = car_y;
	this->car_yaw = car_yaw;
	this->prev_x = prev_x;
	this->prev_y = prev_y;
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_s = map_waypoints_s;
}

double Trajectory::get_d(int lane) {
	return (LANE_WIDTH/2.0)+LANE_WIDTH*lane;
}

/**
 * Builds a spline in the coordinates with origin (x1,y1) and angle yaw.
 * It uses three more points at fixed distance ahead.
 *
 * The spline is in coordinates with origin at (x2,y2) rotated by angle -yaw
 */
tk::spline Trajectory::get_spline( double x1, double y1, double x2, double y2, double yaw, int lane) {

	vector<double> x;
	vector<double> y;

	CoordinateTransformation ct(x1, y1, yaw);

	// add three more points 10m, 20, 30m from car

	vector<double> frenet_coord = getFrenet(x2, y2, yaw, map_waypoints_x, map_waypoints_y);
	double car_s = frenet_coord[0];
	double d = get_d(lane);

	x.push_back(ct.to_x(x1,y1));
	x.push_back(ct.to_x(x2,y2));

	y.push_back(ct.to_y(x1,y1));
	y.push_back(ct.to_y(x2,y2));

	vector<double> wp1 = getXY(car_s+WAYPOINTS_DISTANCE, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> wp2 = getXY(car_s+2*WAYPOINTS_DISTANCE, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> wp3 = getXY(car_s+3*WAYPOINTS_DISTANCE, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	x.push_back(ct.to_x(wp1[0],wp1[1]));
	x.push_back(ct.to_x(wp2[0],wp2[1]));
	x.push_back(ct.to_x(wp3[0],wp3[1]));

	y.push_back(ct.to_y(wp1[0],wp1[1]));
	y.push_back(ct.to_y(wp2[0],wp2[1]));
	y.push_back(ct.to_y(wp3[0],wp3[1]));

	tk::spline s;
	s.set_points(x,y);

	return s;

}

/**
 * yaw in radians
 */
vector<vector<double>> Trajectory::build_trajectory(
					double ego_speed,
					vector <double> prev_x, vector<double> prev_y,
					int target_lane, double target_velocity) {

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	double x1, y1, x2, y2, yaw;

	if ( prev_x.size() < 2 ) {
		x1 = car_x - cos(car_yaw);
		x2 = car_x;
		y1 = car_y - sin(car_yaw);
		y2 = car_y;
		yaw = car_yaw;
	} else {
		x1 = prev_x[prev_x.size()-2];
		y1 = prev_y[prev_y.size()-2];
		x2 = prev_x[prev_x.size()-1];
		y2 = prev_y[prev_y.size()-1];
		yaw = atan2( y2-y1, x2-x1 );
		//store the left over points
		for ( int i = 0; i < prev_x.size(); i++) {
			next_x_vals.push_back(prev_x[i]);
			next_y_vals.push_back(prev_y[i]);
		}
	}

	tk::spline s = get_spline(x1, y1, x2, y2, yaw, target_lane);

	CoordinateTransformation ct(x2, y2, yaw);
	double start_x = ct.to_x(x2,y2);
	int n = next_x_vals.size();

	double target_x = DISTANCE_AHEAD;
	double target_distance = distance( 0, 0, target_x, s(target_x) );

	target_velocity *= 1609.34/3600; // change to m/s
	assert(target_velocity > 0);

	int no_intervals = round(target_distance / DELTA_T / target_velocity);

	assert(no_intervals > 0);

	double x_increment = DISTANCE_AHEAD/no_intervals;

	for (int i = 0; i < N-n; ++i) {
		double next_x = start_x + x_increment * (i+1);
		double next_y = s(next_x);

		next_x_vals.push_back(ct.from_x(next_x, next_y));
		next_y_vals.push_back(ct.from_y(next_x, next_y));

	}

	return {next_x_vals, next_y_vals};
}

vector<vector<double>> Trajectory::build_straight_trajectory() {
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	double dist_inc = 49 * 1609.34 * 0.02 / 3600;
	for (int i = 0; i < 50; ++i) {
		next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
		next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
	}
	return {next_x_vals, next_y_vals};
}
