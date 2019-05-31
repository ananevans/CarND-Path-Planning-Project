/*
 * TrajectoryCostCalculator.h
 *
 *  Created on: May 28, 2019
 *      Author: nora
 */

#ifndef SRC_TRAJECTORYCOSTCALCULATOR_H_
#define SRC_TRAJECTORYCOSTCALCULATOR_H_

#include <vector>

#include "Trajectory.h"
#include "Prediction.h"

using namespace std;

typedef double (*CostFunctionPtr)(Trajectory ego_trajectory, vector<vector<Prediction>> predictions);

#define NO_COST_FN 10


class TrajectoryCostCalculator {
public:

	static double get_cost( Trajectory ego_trajectory, vector<vector<Prediction>> predictions );

private:
	static double WEIGHTS[NO_COST_FN];
	static CostFunctionPtr COST_FUNCTIONS[NO_COST_FN];
};

#endif /* SRC_TRAJECTORYCOSTCALCULATOR_H_ */
