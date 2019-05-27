/*
 * Prediction.cpp
 *
 *  Created on: May 23, 2019
 *      Author: nora
 */
#include <vector>
#include <assert.h>
#include <iostream>
#include <math.h>

#include "Prediction.h"
#include "Trajectory.h"

#include "Constants.h"

Prediction::Prediction() {
	// TODO Auto-generated constructor stub

}

Prediction::~Prediction() {
	// TODO Auto-generated destructor stub
}

PredictedBehavior Prediction::get_behavior() {
	return behavior;
}

double Prediction::get_probability() {
	return probability;
}

std::vector<double> Prediction::get_x() {
	return x;
}

std::vector<double> Prediction::get_y() {
	return y;
}

double gaussian( double mu, double std_dev, double x ) {
	return 1/(std_dev * sqrt(2+M_PI)) * exp(-0.5*((x-mu)/std_dev)*((x-mu)/std_dev));
}


std::vector<Prediction> Prediction::predict(int lane,
		double x, double y, double vx, double vy,
		double s, double d,
		std::vector<double> map_waypoints_x,
		std::vector<double> map_waypoints_y,
		std::vector<double> map_waypoints_s
		) {
	std::vector<Prediction> result;
	double left = lane * LANE_WIDTH;
	double right = (lane+1) * LANE_WIDTH;
	double middle = (lane + 0.5) * LANE_WIDTH;
	double std_dev = 0.75;
	double pl = gaussian( left, std_dev, d);
	if ( lane == 0 ) {
		pl = 0;
	}
	if ( pl < 0.001 ) {
		pl = 0.0;
	}
	double pm = gaussian( middle, std_dev, d);
	if ( pm < 0.001 ) {
		pm = 0.0;
	}
	double pr = gaussian( right, std_dev, d);
	if ( lane == MAX_LANE ) {
		pr = 0;
	}
	if ( pr < 0.001 ) {
		pr = 0.0;
	}
	double sum = pl + pm + pr;
	pl /= sum;
	pm /= sum;
	pr /= sum;
	if ( lane != 0 && pl > 0.0) {
		Prediction prediction;
		prediction.probability = pl;
		prediction.behavior = predict_change_left;
		Trajectory trajectory( x, y, atan2(vy, vx), {}, {},
				map_waypoints_x, map_waypoints_y, map_waypoints_s);
		vector<vector<double>> xy = trajectory.build_trajectory(lane-1, sqrt(vx*vx + vy*vy));
		prediction.x = xy[0];
		prediction.y = xy[1];
		result.push_back(prediction);
	}
	if ( pm > 0.0 ){
		Prediction prediction;
		prediction.probability = pm;
		prediction.behavior = predict_keep_lane;
		Trajectory trajectory( x, y, atan2(vy, vx), {}, {},
				map_waypoints_x, map_waypoints_y, map_waypoints_s);
		vector<vector<double>> xy = trajectory.build_trajectory(lane, sqrt(vx*vx + vy*vy));
		prediction.x = xy[0];
		prediction.y = xy[1];
		result.push_back(prediction);
	}
	if ( lane != MAX_LANE && pr > 0.0 ) {
		Prediction prediction;
		prediction.probability = pr;
		prediction.behavior = predict_change_right;
		Trajectory trajectory( x, y, atan2(vy, vx), {}, {},
				map_waypoints_x, map_waypoints_y, map_waypoints_s);
		vector<vector<double>> xy = trajectory.build_trajectory(lane+1, sqrt(vx*vx + vy*vy));
		prediction.x = xy[0];
		prediction.y = xy[1];
		result.push_back(prediction);
	}
	return result;
}
