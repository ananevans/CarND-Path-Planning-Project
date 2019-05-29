#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "Behavior.h"
#include "Constants.h"
#include "Prediction.h"

#include <math.h>
#include "TrajectoryGenerator.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

	Behavior behavior(map_waypoints_x, map_waypoints_y, map_waypoints_s);

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
				 &map_waypoints_dx,&map_waypoints_dy,&behavior]
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

//					vector<double> next_x_vals;
//					vector<double> next_y_vals;

					cout << "Previous iteration used " << NO_POINTS - previous_path_x.size() << "\n";

					/**
					 * TODO: define a path made up of (x,y) points that the car will visit
					 *   sequentially every .02 seconds
					 */
					// calculate predictions for vehicles within 100m from the ego vehicle
					vector<vector<Prediction>> all_predictions;
					cout << "Calculating predictions ... \n";
					for ( int i = 0; i < sensor_fusion.size(); i++ ) {
						int vehicle_id = sensor_fusion[i][0];
						double x = sensor_fusion[i][1];
						double y = sensor_fusion[i][2];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double s = sensor_fusion[i][5];
						double d = sensor_fusion[i][6];

						if ( d >= 0 && abs(car_s - s) < 100 ) {
							vector<Prediction> predictions =  Prediction::predict(
									x, y, vx, vy, s, d,
									map_waypoints_x, map_waypoints_y, map_waypoints_s);
							for (int p=0; p< predictions.size(); p++) {
								std::cout << vehicle_id
										<< ": probability " << predictions[p].get_probability()
										<< " behavior  " << predictions[p].get_behavior()
										<< "\n";
							}
							if (predictions.size() > 0) {
								all_predictions.push_back(predictions);
							}
						}
					}
					cout << "Calculating predictions ... DONE for " << all_predictions.size() << " vehicles\n" ;
					// calculate behavior
					vector<double> prev_x = previous_path_x;
					vector<double> prev_y = previous_path_y;
					vector<vector<double>> xy = behavior.calculate_behavior(
							car_x, car_y, car_s, car_d,
							car_speed, car_yaw,
							prev_x, prev_y, end_path_s, end_path_d, all_predictions
							);

					std::cout << "car: x:" << car_x << " y:" << car_y
							<< " speed: " << car_speed << " s: " << car_s
							<< " prev size: " << previous_path_x.size()
							<< "\n";

					int target_lane = TARGET_LANE;
					double target_speed = SPEED_LIMIT;


					msgJson["next_x"] = xy[0];
					msgJson["next_y"] = xy[1];

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


