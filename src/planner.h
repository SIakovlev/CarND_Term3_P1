#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
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
	tk::spline s_xs;
	tk::spline s_ys; 

	std::vector<double> map_waypoints_x;
  	std::vector<double> map_waypoints_y;
  	std::vector<double> map_waypoints_s;
	// --------------------------------------------------------------------------

	int lane_n = 1; // lane number: 0 - left lane, 1 - middle late, 2 - right lane
	int goal_lane_n = 1;
  	double target_speed = 0.0; // mph speed limit
  	const double delta_t = 0.02;
    double speed_limit = 49.5;
    std::vector<double> spline_points = {-100, -10, -5, -3, -2, -1, 0, 1, 2, 3, 5, 10, 100};
    int spline_iterator = 0;
    double acceleration = 0.224;
    double decceleration = 0.224;

	// Constructor
	Planner();
	// Destructor (virtual???)
	~Planner();

	void read_data(json& data_obj, std::vector<std::vector<double>> &map_data);


	std::vector<std::vector<double>> generate_trajectory(double goal);
	tk::spline generate_spline(std::vector<double> start, std::vector<double> end);

	std::vector<double> jmt_coefficients(std::vector<double> start, std::vector<double> end, double T);
	double jmt(double t, std::vector<double> alphas);
	


	void set_speed_limit(int value);
	void change_lane(int value);
	void keep_distance(int dist, int id);

	double lane_cost(std::vector<int> dist_range, int lane);
	int get_lane();

	std::vector<int> vehicles_around(std::vector<int> dist_range, bool dir);

};

#endif