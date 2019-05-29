/*
 * Trajectory.cpp
 *
 *  Created on: May 28, 2019
 *      Author: nora
 */

#include "Trajectory.h"

#include "Constants.h"

Trajectory::Trajectory(){
}

Trajectory::Trajectory(vector<double> x,
		vector <double> y,
		vector <double> s,
		vector <double> d,
		double target_speed,
		double target_lane) {
	this->x = x;
	this->y = y;
	this->target_speed = target_speed;
	this->target_lane = target_lane;
}

Trajectory::~Trajectory() {
	// TODO Auto-generated destructor stub
}

int Trajectory::get_lane(double car_d) {
	if (car_d < LANE_WIDTH) {
		return 0;
	} else {
		if (car_d < 2*LANE_WIDTH) {
			return 1;
		} else {
			return 2;
		}
	}
}

double Trajectory::getTargetLane() const {
	return target_lane;
}

double Trajectory::getTargetSpeed() const {
	return target_speed;
}

const vector<double>& Trajectory::getX() const {
	return x;
}

const vector<double>& Trajectory::getY() const {
	return y;
}
