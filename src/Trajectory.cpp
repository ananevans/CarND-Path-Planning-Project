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

#define DISTANCE_AHEAD 30.0
#define WAYPOINTS_DISTANCE 30.0

#define TIME_HORIZON 1

using namespace std;

Trajectory::Trajectory() {
	// TODO Auto-generated constructor stub

}

Trajectory::~Trajectory() {
	// TODO Auto-generated destructor stub
}

Trajectory::Trajectory(double car_x, double car_y, double car_yaw,
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
	return (LANE_WIDTH / 2.0) + LANE_WIDTH * lane;
}

vector<double> Trajectory::JMT(vector<double> &start, vector<double> &end, double T) {

	assert(3 == start.size());
	assert(3 == end.size());

	//This code is copied from the class solution.

	MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			3*T*T, 4*T*T*T,5*T*T*T*T,
			6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			end[1]-(start[1]+start[2]*T),
			end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};

	for(int i = 0; i < C.size(); ++i) {
		result.push_back(C.data()[i]);
	}

	return result;

}

vector<double> evaluate(vector<double> coeff, double t) {
	vector<double> result;
	double t2 = t*t;
	double t3 = t2*t;
	double t4 = t3*t;
	double t5 = t4*t;
	result.push_back( coeff[0] + coeff[1] * t + coeff[2] * t2 + coeff[3] * t3 + coeff[4] * t4 + coeff[5] * t5 );
	result.push_back( coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t2 + 4 * coeff[4] * t3 + 5 * coeff[5] * t4 );
	result.push_back( 2 * coeff[2] + 2 * 3 * coeff[3] * t + 3 * 4 * coeff[4] * t2 + 4 * 5 * coeff[5] * t3 );
	result.push_back( 2 * 3 * coeff[3] +2 * 3 * 4 * coeff[4] * t + 3 * 4 * 5 * coeff[5] * t2 );
	return result;
}

/**
 * Previously calculated values for x, y, speed, and acceleration.
 */
vector<vector<double>> Trajectory::build_trajectory_JMT(
		vector<double> prev_s, vector<double> prev_d,
		vector<double> prev_v_s, vector<double> prev_v_d,
		vector<double> prev_a_s, vector<double> prev_a_d,
		int target_lane, double target_velocity) {
	assert(prev_s.size() > 0);
	assert(prev_s.size() == prev_d.size());
	assert(prev_s.size() == prev_v_s.size());
	assert(prev_s.size() == prev_v_d.size());
	assert(prev_s.size() == prev_a_s.size());
	assert(prev_s.size() == prev_a_d.size());

//	std::cout << "prev_s: "; debug_print_vector(prev_s); std::cout << "\n";
//	std::cout << "prev_v_s: "; debug_print_vector(prev_v_s); std::cout << "\n";
//	std::cout << "prev_a_s: "; debug_print_vector(prev_a_s); std::cout << "\n";

	vector<double> s_i, s_f, d_i, d_f;

	int last_index = prev_s.size() - 1;
	s_i.push_back(prev_s[last_index]);
	s_i.push_back(prev_v_s[last_index]);
	s_i.push_back(prev_a_s[last_index]);
	s_f.push_back(prev_s[last_index] + DISTANCE_AHEAD);
	s_f.push_back(target_velocity);
	s_f.push_back(0.0);
	d_i.push_back(prev_d[last_index]);
	d_i.push_back(prev_v_d[last_index]);
	d_i.push_back(prev_a_d[last_index]);
	d_f.push_back(get_d(target_lane));
	d_f.push_back(0.0);
	d_f.push_back(0.0);



  	vector<double> new_s, new_v_s, new_a_s, new_d, new_v_d, new_a_d;
	vector<vector<double>> result;

	//copy old data
	for (int i = 0; i < prev_s.size(); i++) {
		new_s.push_back( prev_s[i] );
		new_v_s.push_back( prev_v_s[i] );
		new_a_s.push_back( prev_a_s[i] );
		new_d.push_back( prev_d[i] );
		new_v_d.push_back( prev_v_d[i] );
		new_a_d.push_back( prev_a_d[i] );
	}

	//append new data
	bool done = false;
	int factor = 1;
	while (!done) {

//		std::cout << "s_i: "; debug_print_vector(s_i); std::cout << "\n";
//		std::cout << "d_i: "; debug_print_vector(d_i); std::cout << "\n";
//
//		std::cout << "s_f: "; debug_print_vector(s_f); std::cout << "\n";
//		std::cout << "d_f: "; debug_print_vector(d_f); std::cout << "\n";

		vector<double> s_coeff = JMT( s_i, s_f, factor*TIME_HORIZON );
		vector<double> d_coeff = JMT( d_i, d_f, factor*TIME_HORIZON );

//		std::cout << "s coeff: "; debug_print_vector(s_coeff); std::cout << "\n";
//		std::cout << "d coeff: "; debug_print_vector(d_coeff); std::cout << "\n";


		for ( int i = 0; i < N - prev_s.size(); i++ ) {
			double s, v_s, a_s, j_s, d, v_d, a_d, j_d;
			double t = (i+1) * DELTA_T;
			vector<double> s_data = evaluate(s_coeff, (i+1) * DELTA_T);
			vector<double> d_data = evaluate(d_coeff, (i+1) * DELTA_T);
			// check for validity

//			std::cout << "s data: "; debug_print_vector(s_data); std::cout << "\n";
//			std::cout << "d data: "; debug_print_vector(d_data); std::cout << "\n";

			if ( s_data[1] * s_data[1] + d_data[1] + d_data[1] < MAX_VELOCITY_2
					&& s_data[2] * s_data[2] + d_data[2] + d_data[2] < MAX_ACCELERATION_2
					&& s_data[3] * s_data[3] + d_data[3] + d_data[3] < MAX_JERK_2
			) {
				//add to results
				new_s.push_back(s_data[0]);
				new_v_s.push_back(s_data[1]);
				new_a_s.push_back(s_data[2]);
				new_d.push_back(d_data[0]);
				new_v_d.push_back(d_data[1]);
				new_a_d.push_back(d_data[2]);
			} else {
				break;
			}
		}

		if (N == new_s.size()) {
			done = true;
		} else {
			new_s.resize( prev_s.size() );
			new_v_s.resize( prev_s.size() );
			new_a_s.resize( prev_s.size() );

			new_d.resize( prev_s.size() );
			new_v_d.resize( prev_s.size() );
			new_a_d.resize( prev_s.size() );
		}

		factor += 1;
	}

	//return result
	result.push_back( new_s );
	result.push_back( new_v_s );
	result.push_back( new_a_s );

	result.push_back( new_d );
	result.push_back( new_v_d );
	result.push_back( new_a_d );
	return result;
}

/**
 * Builds a spline in the coordinates with origin (x1,y1) and angle yaw.
 * It uses three more points at fixed distance ahead.
 *
 * The spline is in coordinates with origin at (x2,y2) rotated by angle -yaw
 */
tk::spline Trajectory::get_spline(double x1, double y1, double x2, double y2,
		double yaw, int lane) {

	vector<double> x;
	vector<double> y;

	CoordinateTransformation ct(x1, y1, yaw);

	// add three more points 10m, 20, 30m from car

	vector<double> frenet_coord = getFrenet(x2, y2, yaw, map_waypoints_x,
			map_waypoints_y);
	double car_s = frenet_coord[0];
	double d = get_d(lane);

	x.push_back(ct.to_x(x1, y1));
	x.push_back(ct.to_x(x2, y2));

	y.push_back(ct.to_y(x1, y1));
	y.push_back(ct.to_y(x2, y2));

	vector<double> wp1 = getXY(car_s + WAYPOINTS_DISTANCE, d, map_waypoints_s,
			map_waypoints_x, map_waypoints_y);
	vector<double> wp2 = getXY(car_s + 2 * WAYPOINTS_DISTANCE, d,
			map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> wp3 = getXY(car_s + 3 * WAYPOINTS_DISTANCE, d,
			map_waypoints_s, map_waypoints_x, map_waypoints_y);

	x.push_back(ct.to_x(wp1[0], wp1[1]));
	x.push_back(ct.to_x(wp2[0], wp2[1]));
	x.push_back(ct.to_x(wp3[0], wp3[1]));

	y.push_back(ct.to_y(wp1[0], wp1[1]));
	y.push_back(ct.to_y(wp2[0], wp2[1]));
	y.push_back(ct.to_y(wp3[0], wp3[1]));

	tk::spline s;
	s.set_points(x, y);

	return s;

}

/**
 * yaw in radians
 */
vector<vector<double>> Trajectory::build_trajectory(double ego_speed,
		vector<double> prev_x, vector<double> prev_y, int target_lane,
		double target_velocity) {

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	double x1, y1, x2, y2, yaw;

	if (prev_x.size() < 2) {
		x1 = car_x - cos(car_yaw);
		x2 = car_x;
		y1 = car_y - sin(car_yaw);
		y2 = car_y;
		yaw = car_yaw;
	} else {
		x1 = prev_x[prev_x.size() - 2];
		y1 = prev_y[prev_y.size() - 2];
		x2 = prev_x[prev_x.size() - 1];
		y2 = prev_y[prev_y.size() - 1];
		yaw = atan2(y2 - y1, x2 - x1);
		//store the left over points
		for (int i = 0; i < prev_x.size(); i++) {
			next_x_vals.push_back(prev_x[i]);
			next_y_vals.push_back(prev_y[i]);
		}
	}

	tk::spline s = get_spline(x1, y1, x2, y2, yaw, target_lane);

	CoordinateTransformation ct(x2, y2, yaw);
	double start_x = ct.to_x(x2, y2);
	int n = next_x_vals.size();

	double target_x = DISTANCE_AHEAD;
	double target_distance = distance(0, 0, target_x, s(target_x));

	target_velocity *= 1609.34 / 3600; // change to m/s
	assert(target_velocity > 0);

	int no_intervals = round(target_distance / DELTA_T / target_velocity);

	assert(no_intervals > 0);

	double x_increment = DISTANCE_AHEAD / no_intervals;

	for (int i = 0; i < N - n; ++i) {
		double next_x = start_x + x_increment * (i + 1);
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
		next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
		next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
	}
	return {next_x_vals, next_y_vals};
}
