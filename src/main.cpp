#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "planner.h"
//#include "matplotlibcpp.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  bool plot_flag = true;

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

  // Create a behaviour planner for our car
  Planner p;
  h.onMessage([&p, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // Store map data
          std::vector<std::vector<double>> map_data = {map_waypoints_x, map_waypoints_y, map_waypoints_s};
          // Define next xy values vector
          vector<vector<double>> next_xy_vals;

          // Send JSON and map data to the planner
          p.read_data(j[1], map_data);
          
          // Show vehicles ahead and behind of the ego vehicle
          // ----------------------------------------------------
          std::vector<int> distance_range = {20, 10};
          std::vector<int> vehicles_ahead = p.vehicles_around(distance_range, true);
          cout << "Vehicles_ahead: ";
          for (int i = 0; i <= 2; i++) {
            cout << vehicles_ahead[i] << " ";
          }
          std::vector<int> vehicles_behind = p.vehicles_around(distance_range, false);
          cout << "  " << "Vehicles behind: ";
          for (int i = 0; i <= 2; i++) {
            cout << vehicles_behind[i] << " ";
          }
          cout << endl;
          // ----------------------------------------------------
          
          const int num_of_lanes = 2;
          auto current_lane = p.get_lane();
          // Generate next trajectory points
          if (vehicles_ahead[current_lane] != -1) {
            auto vehicle_ahead_id = vehicles_ahead[current_lane];
            if (current_lane == 1) {
              // Assess available lanes
              auto right_lane_cost = p.lane_cost(distance_range, current_lane + 1);
              auto left_lane_cost = p.lane_cost(distance_range, current_lane - 1);
              cout << "Costs: " << "left lane - " << left_lane_cost << "right lane - " << right_lane_cost << endl;

              // Change left or right
              //if (vehicles_ahead[current_lane + 1] == -1) {
              if ((!right_lane_cost) || ((left_lane_cost >= right_lane_cost) && right_lane_cost != 100)) {
                cout << "Lane change to the right" << endl;
                p.change_lane(current_lane + 1);
              } //else if (vehicles_ahead[current_lane - 1]  == -1) {
              else if ((!left_lane_cost) || ((left_lane_cost <= right_lane_cost) && left_lane_cost != 100)) {
                cout << "Lane change to the left" << endl;
                p.change_lane(current_lane - 1);
              } else {
                cout << "Keep distance, id: " << vehicle_ahead_id << endl;
                p.keep_distance(20, vehicle_ahead_id);
              }

            } else if (current_lane == 0) {
              auto middle_lane_cost = p.lane_cost(distance_range, current_lane + 1);
              //if (vehicles_ahead[current_lane + 1] == -1) {
              if (middle_lane_cost < 100) {
                cout << "Lane change to the right" << endl;
                p.change_lane(current_lane + 1);
              } else {
                cout << "Keep distance, id: " << vehicle_ahead_id << endl;
                p.keep_distance(20, vehicle_ahead_id);
              }

            } else if (current_lane == 2) {
              auto middle_lane_cost = p.lane_cost(distance_range, current_lane - 1);
              //if (vehicles_ahead[current_lane - 1] == -1) {
              if (middle_lane_cost < 100) {
                cout << "Lane change to the left" << endl;
                p.change_lane(current_lane - 1);
              } else {
                cout << "Keep distance, id: " << vehicle_ahead_id << endl;
                p.keep_distance(20, vehicle_ahead_id);
              }
            }
          } else {
            p.set_speed_limit(49.5);
          } 
          
          next_xy_vals = p.generate_trajectory(20.0);
          
          json msgJson;
          msgJson["next_x"] = next_xy_vals[0];
          msgJson["next_y"] = next_xy_vals[1];

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
