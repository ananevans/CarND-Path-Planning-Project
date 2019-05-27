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
#include "Prediction.h"

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

	double previous_speed = 0.0;

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
				 &map_waypoints_dx,&map_waypoints_dy,&previous_speed]
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
					for ( int i = 0; i < sensor_fusion.size(); i++ ) {
						int vehicle_id = sensor_fusion[i][0];
						double x = sensor_fusion[i][1];
						double y = sensor_fusion[i][2];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double s = sensor_fusion[i][5];
						double d = sensor_fusion[i][6];
						int lane = get_lane(d);

						if ( d >= 0 && abs(car_s - s) < 100 ) {
							vector<Prediction> predictions =  Prediction::predict(
									lane, x, y, vx, vy, s, d,
									map_waypoints_x, map_waypoints_y, map_waypoints_s);
							for (int p=0; p< predictions.size(); p++) {
								std::cout << vehicle_id
										<< ": probability " << predictions[p].get_probability()
										<< " behavior  " << predictions[p].get_behavior()
										<< "\n";
							}
						}
					}


					int current_lane = get_lane( car_d );
					double future_s, t, future_velocity, future_lane;
					if (previous_path_x.size() > 1) {
						// use last generated values
						double y2 = previous_path_y[previous_path_y.size() - 1] ;
						double y1 = previous_path_y[previous_path_y.size() - 2];
						double x2 = previous_path_x[previous_path_x.size() - 1];
						double x1 = previous_path_x[previous_path_x.size() - 2];
						double yaw = atan2( y2 - y1, x2 - x1 );
						vector<double> frenet_coordinates = getFrenet(
								previous_path_x[previous_path_x.size() - 1],
								previous_path_y[previous_path_y.size() - 1],
								yaw,
								map_waypoints_x, map_waypoints_y
								);
						future_s = frenet_coordinates[0];
						future_lane = get_lane( frenet_coordinates[1] );
						future_velocity = previous_speed;
						t = DELTA_T * previous_path_x.size();
					} else {
						// use defaults
						future_s = car_s;
						future_velocity = car_speed;
						t = 0.0;
						future_lane = get_lane(car_d);
					}

					Behavior behavior(map_waypoints_x, map_waypoints_y, map_waypoints_s);
					int next = behavior.calculate_behavior(future_s, future_lane, future_velocity, t,
							sensor_fusion);

					std::cout << "car: x:" << car_x << " y:" << car_y
							<< " speed: " << car_speed << " s: " << car_s
							<< " prev size: " << previous_path_x.size()
							<< "\n";

					int target_lane = TARGET_LANE;
					double target_speed = SPEED_LIMIT;

					switch (next) {
					case change_left: {
						assert(current_lane > 0);
						target_lane = current_lane - 1;
						std::cout<<"left\n";
						break;
					}
					case change_right: {
						assert(current_lane<MAX_LANE);
						target_lane = current_lane + 1;
						std::cout<<"right\n";
						break;
					}
					case keep_lane: {
						target_speed = min (previous_speed+DELTA_VELOCITY_UP, SPEED_LIMIT);
						target_lane = current_lane;
						previous_speed = target_speed;
						std::cout << "Increasing speed to " << target_speed << "\n";
						break;
						// TODO decide increase speed or slow down
					}
//					case Behavior::DECREASE_SPEED:
//
//						if ( behavior.get_closest_car_speed() >  previous_speed-DELTA_VELOCITY_DOWN ) {
//							target_speed = behavior.get_closest_car_speed();
//						} else {
//							target_speed = max(previous_speed-DELTA_VELOCITY_DOWN, 0.0);
//						}
//						target_lane = current_lane;
//						previous_speed = target_speed;
//						std::cout << "Decreasing speed to " << target_speed << "\n";
//						break;
					default:
						target_speed = car_speed;
						target_lane = current_lane;
						break;
					}

					// if the target speed is within a small range from the previous one, do not change speed
					//TODO use the distance information
					if ( abs(target_speed - previous_speed) < 1 ) {
						target_speed = previous_speed;
					}


					Trajectory trajectory(
							car_x, car_y, deg2rad(car_yaw),
							previous_path_x, previous_path_y,
							map_waypoints_x, map_waypoints_y, map_waypoints_s);
					vector<vector<double>> res = trajectory.build_trajectory(
							target_lane, target_speed);
					next_x_vals = res[0];
					next_y_vals = res[1];

//					vector<double> speeds_x, speeds_y;
//					for ( int i = 0; i < next_x_vals.size() - 1; i++) {
//						double vx = (next_x_vals[i+1] - next_x_vals[i])/DELTA_T;
//						double vy = (next_y_vals[i+1] - next_y_vals[i])/DELTA_T;
//						speeds_x.push_back(vx);
//						speeds_y.push_back(vy);
//						if ( sqrt(vx*vx+vy*vy) >= SPEED_LIMIT_MPS ) {
//							std::cout << "Speed Violation: " << sqrt(vx*vx+vy*vy)
//									<< " index "<< i <<"\n";
//						}
//					}
//					vector<double> acceleration_x;
//					vector<double> acceleration_y;
//					for ( int i = 0; i < speeds_x.size() - 1; i++ ) {
//						double ax = (speeds_x[i+1] - speeds_x[i])/DELTA_T;
//						double ay = (speeds_y[i+1] - speeds_y[i])/DELTA_T;
//						acceleration_x.push_back( ax );
//						acceleration_y.push_back( ay );
//						if ( sqrt(ax*ax+ay*ay) >= MAX_ACCELERATION ) {
//							std::cout << "Acceleration Violation: " << sqrt(ax*ax+ay*ay)
//									<< " index "<< i <<"\n";
//						}
//					}
//					for ( int i = 0; i < acceleration_x.size() - 1; i++ ) {
//						double jx = (acceleration_x[i+1] - acceleration_x[i])/DELTA_T;
//						double jy = (acceleration_y[i+1] - acceleration_y[i])/DELTA_T;
//						if ( sqrt(jx*jx+jy*jy) >= MAX_JERK ) {
//							std::cout << "Jerk Violation: " << sqrt(jx*jx+jy*jy) << " index "<< i <<"\n";
//						}
//					}


					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

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


