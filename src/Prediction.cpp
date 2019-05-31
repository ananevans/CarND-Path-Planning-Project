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
#include "Constants.h"
#include "TrajectoryGenerator.h"
#include "Trajectory.h"

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

double gaussian( double mu, double std_dev, double x ) {
	return 1/(std_dev * sqrt(2+M_PI)) * exp(-0.5*((x-mu)/std_dev)*((x-mu)/std_dev));
}

/**
 * Naive Bayes Algorithm to determine the probability of a behavior (keep lane,
 * change left, change right) given the d coordinate. I assume the three states have equal
 * probability. If the states have different probabilities then the variables pl, pm and pr
 * should be multiplied by the corresponding probabilities of change left, keep lane and change
 * right.
 * For the probability p(d|state) I use a Gaussian distribution with mean on the left side,
 * middle of the lane and right side and a standard deviation of 0.75 for the sides and 1 for the center.
 */
std::vector<Prediction> Prediction::predict(
		double x, double y, double vx, double vy,
		double s, double d,
		std::vector<double> map_waypoints_x,
		std::vector<double> map_waypoints_y,
		std::vector<double> map_waypoints_s
		) {
	int lane = Trajectory::get_lane(d);
	std::vector<Prediction> result;
	double left = lane * LANE_WIDTH;
	double right = (lane+1) * LANE_WIDTH;
	double middle = (lane + 0.5) * LANE_WIDTH;
	double pl = gaussian( left, 0.75, d);
	if ( lane == 0 ) {
		pl = 0;
	}
	if ( pl < 0.1 ) {
		pl = 0.0;
	}
	double pm = gaussian( middle, 1, d);
	if ( pm < 0.1 ) {
		pm = 0.0;
	}
	double pr = gaussian( right, 0.75, d);
	if ( lane == MAX_LANE ) {
		pr = 0;
	}
	if ( pr < 0.1 ) {
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
		TrajectoryGenerator gen( x, y, atan2(vy, vx), {}, {},
				map_waypoints_x, map_waypoints_y, map_waypoints_s);
		prediction.trajectory = gen.build_trajectory(lane-1, lane-1, sqrt(vx*vx + vy*vy));
		result.push_back(prediction);
	}
	if ( pm > 0.0 ){
		Prediction prediction;
		prediction.probability = pm;
		prediction.behavior = predict_keep_lane;
		TrajectoryGenerator gen( x, y, atan2(vy, vx), {}, {},
				map_waypoints_x, map_waypoints_y, map_waypoints_s);
		prediction.trajectory = gen.build_trajectory(lane, lane, sqrt(vx*vx + vy*vy));
		result.push_back(prediction);
	}
	if ( lane != MAX_LANE && pr > 0.0 ) {
		Prediction prediction;
		prediction.probability = pr;
		prediction.behavior = predict_change_right;
		TrajectoryGenerator gen( x, y, atan2(vy, vx), {}, {},
				map_waypoints_x, map_waypoints_y, map_waypoints_s);
		prediction.trajectory =  gen.build_trajectory(lane+1, lane+1, sqrt(vx*vx + vy*vy));
		result.push_back(prediction);
	}
	return result;
}
