# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Implementation

The goal of this project is to generate trajectories that obey the speed limit, are comfortable (acceleration less than 10m/s^2 and jerk smaller than 10m/s^3), change lanes when stuck behind a slow car, with speed as close as possible to the speed limit.

The sensor fusion data and the car's position and speed are provided by the simulator. The trajectory is a set of 50 points, visited each 20ms by the car. 

The general algorith follows the steps below:
1. generate predictions for the other traffic participants. I used a Naive Bayes Algorithm to determine the probability of the following behavior: keep lane, change lane left, change lane right (see [src/Prediction.cpp](https://github.com/ananevans/CarND-Path-Planning-Project/blob/master/src/Prediction.cpp)).

2. using the current state of the Finite State Machine below, we build possible trajectories for the ego vehicle
![FSM](https://github.com/ananevans/CarND-Path-Planning-Project/blob/master/images/fsm.png)

For the keep lane, and prepare to change lane, we generate a trajectory of increased speed, the same speed and three slower trajectories. For the change lane one trajectory with the same speed is generated.

The trajectories are built using the recommended spline library.The trajectory is generated here: [src/TrajectoryGenerator.cpp](https://github.com/ananevans/CarND-Path-Planning-Project/blob/master/src/TrajectoryGenerator.cpp). I used the method presented in the Project Q&A video.

3. Choose the trajectory of minimum cost. The cost is calculated in [src/TrajectoryCostCalculator.cpp](https://github.com/ananevans/CarND-Path-Planning-Project/blob/master/src/TrajectoryCostCalculator.cpp).

I implemented the following cost functions:
* collision_cost: return 1.0 if a colision is detected between the ego vehicle and one of the predicted trajectories, otherwise it reurns 0.0.
* road_limits: returns 1.0 id the d coordinate is negative or higher than 12.0, otherwise it returns 0.0
* speed_limit_violation: returns 1.0 if the target velocity of the trajectory is higher than the speed limit.
* lane_speed_match: the closer the target speed is to the target lane speed, the lower the cost. This function helps choose the fastest trajectory on the same lane.
* buffer_cost: if there is no vehicle within 2 seconds, then it returns 0.0. If there is a vehicle within 2 seconds, then the cost is higher if the vehicle is closer.
* gap_cost: if the final lane of the trajectory is the same with the target lane, then this cost is 0.0. Otherwise is smaller as the gap is bigger. The cost is 0.0 if there are no vehicles within 2 seconds from the ego vehicle. 
* lane_crossing_cost: I introduced this cost to penalize trajectories that osscilate between two lanes
* target_lane_cost: I introduced this cost to return to the center lane when everything else is equal.
* inefficinecy_cost: I introduced this cost to encourage lane changes when the other lane is faster.

4. Pass the vectors of x and y coordinates to the simulator.

Below is a screenshot of one run that satifies the conditions of the project:
![Run](https://github.com/ananevans/CarND-Path-Planning-Project/blob/master/images/run.png)

### Known Issues

The prediction is based on the current vehicle speed. I assume the vehicle will continue at the current speed. I observed some runs with cut ins very close to the ego vehicle, followed by a quick decelleration. In this case a collision is unavoidable, because the position where the collosion occurs was already generated. To mitigate this, I made the gap_cost dependent inversely on the size of the gap and I increased the likelyhood of a lane change in prediction.

Future work: implement a better prediction algorithm that accounts for acceleration and decelleration of the vehicle.
