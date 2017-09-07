#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "realize.h"
#include "cost_functions.h"
#include "generate_trajectories.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  Trajectory t;
  Realize r;
  Cost c;

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // start in lane 1. Other options 0,1,2
  int lane = 1;

  // reference velocity (mph) to target. Start with a speed of 0 mph.
  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane, &t, &r, &c](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
	  
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

			// previous path size
			int prev_size = previous_path_x.size();

			if (prev_size > 0)
			{
				car_s = end_path_s;
			}

			bool too_close = false;

			// Which lane is my car in?
			lane = car_d/4;

			// Vector of possible states
			vector<string> states = { "KL", "LCL", "RCL"};

			// declare cost values
			double cost_KL = 0;
			double cost_LCL = 0;
			double cost_RCL = 0;

			// Elimiate unfeasible states
			if (lane == 0) 
			{
				// car is in left most lane
				states = { "KL", "RCL" };
				cost_LCL = 1;
				cout << states[0] << states[1] << endl;
			}
			else if (lane == 1)
			{
				// car is in middle lane
				states = { "KL", "LCL", "RCL" };
				cout << states[0] << states[1] << states[2] << endl;
			}
			else if (lane == 2)
			{
				// car is in right most lane
				states = { "KL", "LCL" };
				cost_RCL = 1;
				cout << states[0] << states[1] << endl;
			}


			// declare min ids
			int min_car_id;
			vector<int> left_car_id;
			vector<int> right_car_id;

			for (int i = 0; i < states.size(); i++) {
				if (states[i] == "KL") {
					// realize keep lane
					min_car_id = r.car_in_my_lane(sensor_fusion, lane, car_s, prev_size);
					// compute cost for trajectory
					if (min_car_id >= 0 && min_car_id < 99) 
					{
						double other_car_s = sensor_fusion[min_car_id][5];
						double other_car_d = sensor_fusion[min_car_id][6];
						double other_car_vx = sensor_fusion[min_car_id][3];
						double other_car_vy = sensor_fusion[min_car_id][4];
						double other_car_v = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
						cost_KL = 0.85*c.collision_cost(car_s, other_car_s) + 0.15*c.time_to_collide(car_s, other_car_s, car_speed, other_car_v); // takes input in mph 
					}
					else if (min_car_id == 99)
					{
						cost_KL = 0;
					}
					cout << "KL: " << cost_KL << endl;
				}
				else if (states[i] == "LCL") {
					// Find a car in left lane
					left_car_id = r.car_in_left_lane(sensor_fusion, lane, car_s, prev_size);
					
					// compute cost for trajectory
					if (left_car_id[0] != 99) 
					{
						double local_min_diff = 999.9;
						double local_min_id = 0;
						for (int j = 0; j < left_car_id.size(); j++)
						{
							double other_car_s = sensor_fusion[left_car_id[j]][5];
							if (fabs(car_s - other_car_s) < local_min_diff)
							{
								local_min_diff = fabs(car_s - other_car_s);
								local_min_id = left_car_id[j];
							}
						}
						double other_car_s = sensor_fusion[local_min_id][5];
						double other_car_d = sensor_fusion[local_min_id][6];
						double other_car_vx = sensor_fusion[local_min_id][3];
						double other_car_vy = sensor_fusion[local_min_id][4];
						double other_car_v = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
						cost_LCL = 0.3*c.collision_cost(car_s, other_car_s) + 0.1*c.time_to_collide(car_s, other_car_s, car_speed, other_car_v);
						double look_ahead_time = 0.02 * 100.0;
						other_car_s += other_car_v * look_ahead_time;
						cost_LCL += 0.6*c.collision_cost(car_s, other_car_s);
					}
					
					else if (left_car_id[0] == 99)
					{
						cost_LCL = 0.0;
					}
					cout << "LCL: " << cost_LCL << endl;
				}
				else if (states[i] == "RCL") {
					// Find a car in left lane
					right_car_id = r.car_in_right_lane(sensor_fusion, lane, car_s, prev_size);

					// compute cost for trajectory
					if (right_car_id[0] != 99)
					{
						double local_min_diff = 999.9;
						double local_min_id = 0;
						for (int j = 0; j < right_car_id.size(); j++)
						{
							double other_car_s = sensor_fusion[right_car_id[j]][5];
							if (fabs(car_s - other_car_s) < local_min_diff)
							{
								local_min_diff = fabs(car_s - other_car_s);
								local_min_id = right_car_id[j];
							}
						}
						double other_car_s = sensor_fusion[local_min_id][5];
						double other_car_d = sensor_fusion[local_min_id][6];
						double other_car_vx = sensor_fusion[local_min_id][3];
						double other_car_vy = sensor_fusion[local_min_id][4];
						double other_car_v = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
						cost_RCL = 0.3*c.collision_cost(car_s, other_car_s) + 0.1*c.time_to_collide(car_s, other_car_s, car_speed, other_car_v);
						double look_ahead_time = 0.02 * 100.0;
						other_car_s += other_car_v * look_ahead_time;
						cost_RCL += 0.6*c.collision_cost(car_s, other_car_s);
					}

					else if (right_car_id[0] == 99)
					{
						cost_RCL = 0.0;
					}
					cout << "RCL: " << cost_RCL << endl;
				}
			}
			
			double hyst = 0.1;

			// find minimum cost
			if ((cost_LCL + hyst < cost_KL && cost_LCL + hyst< cost_RCL && cost_LCL < 0.75) || (cost_KL != 0 && cost_LCL == cost_RCL && cost_LCL == 0))
			{
				// realize a left lane change event
				lane = r.realize_left_lane_change(lane);
				
			}
			else if((cost_KL + hyst < cost_LCL && cost_KL + hyst < cost_RCL) || (cost_KL > 0.7 && cost_LCL > 0.6 && cost_RCL > 0.6))
			{
				// realize a keep lane event
				ref_vel = r.realize_keep_lane(sensor_fusion, min_car_id, ref_vel, car_s, prev_size);
			
			}
			else if (cost_RCL + hyst < cost_KL && cost_RCL + hyst < cost_LCL && cost_RCL < 0.75)
			{
				// realize a right lane change event
				lane = r.realize_right_lane_change(lane);

			}
			else if(cost_KL == cost_LCL || cost_KL == cost_RCL)
			{
				// realize a keep lane event
				ref_vel = r.realize_keep_lane(sensor_fusion, min_car_id, ref_vel, car_s, prev_size);
			}

			json msgJson;

			vector<vector<double>> next_xy_vals;
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			next_xy_vals = t.create_trajectory(car_x, car_y, car_yaw, car_s, lane, ref_vel, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			next_x_vals = next_xy_vals[0];
			next_y_vals = next_xy_vals[1];
			// END
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
















































































