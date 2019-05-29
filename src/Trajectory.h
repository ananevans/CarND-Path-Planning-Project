/*
 * Trajectory.h
 *
 *  Created on: May 28, 2019
 *      Author: nora
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#include <vector>

using namespace std;

class Trajectory {
public:
	Trajectory();
	Trajectory(vector<double> x,
			vector <double> y,
			vector <double> s,
			vector <double> d,
			double target_speed,
			int target_lane);
	virtual ~Trajectory();

	static int get_lane(double car_d);
	int getTargetLane() const;
	double getTargetSpeed() const;
	const vector<double>& getX() const;
	const vector<double>& getY() const;

	const vector<double>& getD() const {
		return d;
	}

	const vector<double>& getS() const {
		return s;
	}

private:
	vector<double> x;
	vector <double> y;
	vector<double> s;
	vector <double> d;
	double target_speed;
	double target_lane;
};

#endif /* SRC_TRAJECTORY_H_ */
