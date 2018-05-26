#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "json.hpp"
#include "spline.h"
#include "helper.h"

#define DEBUG 0

using json = nlohmann::json;

class Planner {
public:
	/*
	 *  JSON data
	 * --------------------------------------------------------------------------
	 */
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
	// Previous path data given to the Planner
	json previous_path_x;
	json previous_path_y;
	// Previous path's end s and d values 
	double end_path_s;
	double end_path_d;
	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	json sensor_fusion;
	// --------------------------------------------------------------------------

	// Map data 
	std::vector<double> map_waypoints_x;
  	std::vector<double> map_waypoints_y;
  	std::vector<double> map_waypoints_s;
  	std::vector<double> map_waypoints_dx;
  	std::vector<double> map_waypoints_dy;

	// --------------------------------------------------------------------------

	int lane_n = 1; // lane number: 0 - left lane, 1 - middle late, 2 - right lane
	int goal_lane_n = 1;
  	double target_speed = 0.0; // mph speed limit
  	const double delta_t = 0.02;
    double speed_limit = 49.5;


	/*
	int lane;
	int s;
	float v;
	float a;
	float target_speed;
	int lanes_available;
	float max_acceleration;
	int goal_lane;
	int goal_s;
	string state;
	*/

	// Constructor
	Planner();
	// Destructor (virtual???)
	~Planner();

	void read_data(json& data_obj, std::vector<std::vector<double>>& map_data);


	std::vector<std::vector<double>> generate_trajectory(int goal);


	void set_speed_limit(int value);
	void change_lane(int value);

	int get_lane();

	std::vector<bool> vehicles_around(int dist, bool dir);
};

#endif