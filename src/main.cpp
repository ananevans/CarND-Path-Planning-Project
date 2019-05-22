#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "Trajectory.h"
#include "Behavior.h"
#include "Constants.h"

#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int get_lane(double car_d) {
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



int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	vector<double> prev_v_x, prev_a_x, prev_v_y, prev_a_y;
	double target_velocity = 0.0;

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
				 &map_waypoints_dx,&map_waypoints_dy,
				 &prev_v_x, &prev_a_x,
				 &prev_v_y, &prev_a_y,
				 &target_velocity
				 ]
				 (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
						 uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					/**
					 * TODO: define a path made up of (x,y) points that the car will visit
					 *   sequentially every .02 seconds
					 */
					// drop the consumed values
					int n = previous_path_x.size();

					int delta = prev_v_x.size() - n;
					for (int i = delta; i < prev_v_x.size(); i++) {
						prev_v_x[i-delta] = prev_v_x[i];
						prev_a_x[i-delta] = prev_a_x[i];
						prev_v_y[i-delta] = prev_v_y[i];
						prev_a_y[i-delta] = prev_a_y[i];
					}
					prev_v_x.resize(n);
					prev_a_x.resize(n);
					prev_v_y.resize(n);
					prev_a_y.resize(n);


//					double previous_speed;
//					if ( n > 0) {
//						previous_speed = sqrt( prev_v_x[n-1] * prev_v_x[n-1] + prev_v_y[n-1] * prev_v_y[n-1] );
//					} else {
//						previous_speed = 0.0;
//					}

					int current_lane = get_lane( car_d );
					Behavior behavior(
							car_s, current_lane, car_speed, sensor_fusion,
							map_waypoints_x, map_waypoints_y, map_waypoints_s);
					int next = behavior.calculate_behavior();

					std::cout << "car: x:" << car_x << " y:" << car_y
							<< " speed: " << car_speed << " s: " << car_s
							<< " prev size: " << previous_path_x.size()
							<< "\n";

					int target_lane = TARGET_LANE;

					switch (next) {
					case Behavior::CHANGE_LANE_LEFT: {
						assert(current_lane > 0);
						target_lane = current_lane - 1;
						std::cout<<"left\n";
						break;
					}
					case Behavior::CHANGE_LANE_RIGHT: {
						assert(current_lane<MAX_LANE);
						target_lane = current_lane + 1;
						std::cout<<"right\n";
						break;
					}
					case Behavior::INCREASE_SPEED: {
						//target_velocity = min (target_velocity+DELTA_VELOCITY_UP, SPEED_LIMIT_MPS);
						target_velocity = SPEED_LIMIT_MPS;
						target_lane = current_lane;
						std::cout << "Increasing speed to " << target_velocity << "\n";
						break;
					}
					case Behavior::DECREASE_SPEED:
						target_velocity = max(target_velocity-DELTA_VELOCITY_DOWN, 0.0);
						target_lane = current_lane;
						std::cout << "Decreasing speed to " << target_velocity << "\n";
						break;
					default:
						target_velocity = car_speed;
						target_lane = current_lane;
						break;
					}

					std::cout << "Calculating trajectory...\n";

					Trajectory trajectory(
							car_x, car_y, deg2rad(car_yaw),
							previous_path_x, previous_path_y,
							map_waypoints_x, map_waypoints_y, map_waypoints_s);
					vector<vector<double>> res = trajectory.build_trajectory_JMT(
							prev_v_x, prev_v_y,
							prev_a_x, prev_a_y,
							target_lane, target_velocity);

					std::cout << "Trajectory calculated...\n";

					//copy the tail from res

					int start = previous_path_x.size();
					for ( int i = start; i < N; i++ ) {
						prev_v_x.push_back(res[1][i]);
						prev_a_x.push_back(res[2][i]);
						prev_v_y.push_back(res[4][i]);
						prev_a_y.push_back(res[5][i]);
					}

					std::cout << "target speed: " << target_velocity << " last speed: "
							<< sqrt( prev_v_x[prev_v_x.size()-1] * prev_v_x[prev_v_x.size()-1]
									+  prev_v_y[prev_v_y.size()-1] * prev_v_y[prev_v_y.size()-1] )
							<< "\n";

					msgJson["next_x"] = res[0];
					msgJson["next_y"] = res[3];

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}



