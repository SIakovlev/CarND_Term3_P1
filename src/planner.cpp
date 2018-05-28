// Planner implementation
#include "planner.h"

Planner::Planner(){}
Planner::~Planner() {}

void Planner::read_data(json& data_obj, std::vector<std::vector<double>>& map_data) {

	// JSON data
  this->car_x = data_obj["x"];
	this->car_y = data_obj["y"];
	this->car_s = data_obj["s"];
	this->car_d = data_obj["d"];
	this->car_yaw = data_obj["yaw"];
	this->car_speed = data_obj["speed"];
	// Previous path data given to the Planner
	this->previous_path_x = data_obj["previous_path_x"];
	this->previous_path_y = data_obj["previous_path_y"];
	// Previous path's end s and d values 
	this->end_path_s = data_obj["end_path_s"];
	this->end_path_d = data_obj["end_path_d"];
	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	this->sensor_fusion = data_obj["sensor_fusion"];

	// Map data
	this->map_waypoints_x = map_data[0];
	this->map_waypoints_y = map_data[1];
	this->map_waypoints_s = map_data[2];
	this->map_waypoints_dx = map_data[3];
	this->map_waypoints_dy = map_data[4];
}

std::vector<std::vector<double>> Planner::generate_spline_points() {
  int lane_d = 2+4*lane_n;
  int goal_lane_d = 2+4*goal_lane_n;
  int dist_inc = 50;
  static bool turning_flag = false;
  if ((lane_n == goal_lane_n) && (!turning_flag)) {
    auto p0 = getXY(car_s + dist_inc, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    auto p1 = getXY(car_s + 2*dist_inc, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    auto p2 = getXY(car_s + 3*dist_inc, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    return {p0, p1, p2};
  } else {
    turning_flag = true;
    lane_n = goal_lane_n;
    auto p0 = getXY(car_s + dist_inc, lane_d + (goal_lane_d - lane_d)*logistic(spline_points[spline_iterator++]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    auto p1 = getXY(car_s + 2*dist_inc, lane_d + (goal_lane_d - lane_d)*logistic(spline_points[spline_iterator++]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    auto p2 = getXY(car_s + 3*dist_inc, lane_d + (goal_lane_d - lane_d)*logistic(spline_points[spline_iterator++]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    if (spline_iterator > spline_points.size()) {
      spline_iterator = 0;
      turning_flag = false;
    }
    return {p0, p1, p2};
  }
}

std::vector<std::vector<double>> Planner::generate_trajectory(int goal) {
	// Define spline points
    std::vector<double> ptsx;
    std::vector<double> ptsy;
    // Size of the previous path 
    int prev_size = previous_path_x.size();
    // Global car coordinates in reference frame
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);   

    // start
    if (prev_size < 2) {
      double car_x_prev = car_x - cos(ref_yaw) * delta_t;
      double car_y_prev = car_y - sin(ref_yaw) * delta_t;
      ptsx.insert(ptsx.end(), {car_x_prev, car_x});
      ptsy.insert(ptsy.end(), {car_y_prev, car_y});
    } else {
      // redefine reference points
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];
      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
      ptsx.insert(ptsx.end(), {ref_x_prev, ref_x});
      ptsy.insert(ptsy.end(), {ref_y_prev, ref_y});
    }
    
    // Add some more points to the spline
    /*
    int lane_d = 2+4*lane_n;
    int goal_lane_d = 2+4*goal_lane_n;
    int dist_inc = 50;
    std::vector<double> next_wp0;
    std::vector<double> next_wp1;
    std::vector<double> next_wp2;

    if (goal_lane_n != lane_n) {
      next_wp0 = getXY(car_s + dist_inc, (double)lane_d * 0.8 + (double)goal_lane_d * 0.2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      next_wp1 = getXY(car_s + 2*dist_inc, (double)lane_d * 0.5 + (double)goal_lane_d * 0.5, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      next_wp2 = getXY(car_s + 3*dist_inc, (double)lane_d * 0.3 + (double)goal_lane_d * 0.7, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      lane_n = goal_lane_n;
    } else {
      next_wp0 = getXY(car_s + dist_inc, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      next_wp1 = getXY(car_s + 2*dist_inc, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      next_wp2 = getXY(car_s + 3*dist_inc, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    }
    */
    std::vector<std::vector<double>> next_wp = generate_spline_points();
    ptsx.insert(ptsx.end(), {next_wp[0][0], next_wp[1][0], next_wp[2][0]});
    ptsy.insert(ptsy.end(), {next_wp[0][1], next_wp[1][1], next_wp[2][1]});

    // Convert point to the local frame
    for(int i = 0; i < ptsx.size(); i++)
    {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x * cos(0.0 - ref_yaw) - shift_y * sin(0.0 - ref_yaw));
      ptsy[i] = (shift_x * sin(0.0 - ref_yaw) + shift_y * cos(0.0 - ref_yaw));
    }

    
    if (target_speed < speed_limit) {
      	target_speed += acceleration;
      // cout << "New reference speed: " << target_speed << endl;
    } else {
    	target_speed -= decceleration;
    }
    
    #if DEBUG 
	    cout << "x-points: " << endl;
	    for(int i = 0; i < ptsx.size(); i++) {
	      cout << ptsx[i] << endl;
	    } 

	    cout << "y-points: " << endl;
	    for(int i = 0; i < ptsy.size(); i++) {
	      cout << ptsy[i] << endl;
	    }
    #endif

    tk::spline s;
    s.set_points(ptsx, ptsy);

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    for (int i = 0; i < prev_size; i++) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = goal;
    double target_y = s(target_x);
    double target_dist = distance(0.0, 0.0, target_x, target_y);

    double x_add_on = 0.0;

    for (int i = 0; i <= 50 - prev_size; i++) {
      double N = target_dist/(0.02 * target_speed/2.24);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);

      x_add_on = x_point;
      double x_ref = x_point;
      double y_ref = y_point;

      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
      x_point += ref_x;
      y_point += ref_y;
      
      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }

    return {next_x_vals, next_y_vals};
}

void Planner::set_speed_limit(int value) {
	this->speed_limit = value;
}

void Planner::change_lane(int value) {
	this->goal_lane_n = value;
}

int Planner::get_lane() {
	return lane_n;
}

std::vector<int> Planner::vehicles_around(std::vector<int> dist_range, bool dir) {
	//
	// dist is a distance to vehicle in direction dir (in meters)
	// dir = true - ahead
	// dir = false - behind
	//
	//std::vector<bool> vehicles_found = {false, false, false};

  std::vector<int> vehicles_found = {-1, -1, -1};
	for (int i = 0; i < sensor_fusion.size(); i++) {
		double vehicle_d = sensor_fusion[i][6];
		double vehicle_s = sensor_fusion[i][5];
		double vy = sensor_fusion[i][4];
		double vx = sensor_fusion[i][3];
		double vehicle_speed = sqrt(vx*vx + vy*vy);
		double vehicle_s_next = vehicle_s + (double)(previous_path_x.size()) * delta_t * vehicle_speed;
		// check all three lanes
		for (int lane = 0; lane <= 2; lane++) {
			// check if there is a vehicle on the lane
			if ((vehicle_d >= 4*lane) && (vehicle_d <= 4*lane + 4)) {
				// if we're interested in vehicles in front of us
				if (dir) {
          int closest_dist = 100;
					if ((vehicle_s_next > end_path_s) && ((vehicle_s_next - end_path_s) < dist_range[0])) {
            if ((vehicle_s_next - end_path_s) < closest_dist) {
              vehicles_found[lane] = sensor_fusion[i][0];
              closest_dist = (vehicle_s_next - end_path_s);
              set_speed_limit(vehicle_speed*2.24);
            } 
          }
				} 
				// if we are interested in vehicles behind us
				else { 
					if ((vehicle_s_next < end_path_s) && ((end_path_s - vehicle_s_next) < dist_range[1]))
						vehicles_found[lane] = sensor_fusion[i][0];
				}
			}
		}
	}
	return vehicles_found;
}

double Planner::lane_cost(std::vector<int> dist_range, int lane) {
	int counter = 0;
  int cost = 0;
	double avg_speed = 0.0;
  cout << "Cost calculation process, lane: " << lane << endl;
	for (int i = 0; i < sensor_fusion.size(); i++) {
		double vehicle_d = sensor_fusion[i][6];
		// if there is a vehicle on the lane
		if ((vehicle_d >= 4*lane) && (vehicle_d <= 4*lane + 4)) {

			double vehicle_s = sensor_fusion[i][5];
			double vy = sensor_fusion[i][4];
			double vx = sensor_fusion[i][3];
			double vehicle_speed = sqrt(vx*vx + vy*vy);
			double vehicle_s_next = vehicle_s + (double)(previous_path_x.size()) * delta_t * vehicle_speed;

      // if there is a vehicle behind or ahead within a close range, then the lane change is forbidden
      if (((vehicle_s_next < end_path_s) && ((end_path_s - vehicle_s_next) < dist_range[1])) || 
        ((vehicle_s_next > end_path_s) && ((vehicle_s_next - end_path_s) < dist_range[0]))) {
        cout << "Lane change is dangerous" << endl;
        return 100;
      } else {
        cout << "Lane change is possible. Calculating cost" << endl;
        // count it
        counter ++;
        // calc average speed
        avg_speed += 1.0/counter * (vehicle_speed - avg_speed);
        cost = (int)avg_speed;
      }
		}
	}
  cout << "Resulting cost: " << cost << endl;
  return cost;
}


void Planner::keep_distance(int dist, int id) {
  // find the vehicle with given id
  int vehicle_num = -1;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    if (sensor_fusion[i][0] == id) {
      vehicle_num = i;
      break;
    }
  }
  cout << "Keep distance routine" << endl;
  double vehicle_s = sensor_fusion[vehicle_num][5];
  double vy = sensor_fusion[vehicle_num][4];
  double vx = sensor_fusion[vehicle_num][3];
  double vehicle_speed = sqrt(vx*vx + vy*vy);
  double vehicle_s_next = vehicle_s + (double)(previous_path_x.size()) * delta_t * vehicle_speed;
  if (fabs(vehicle_s_next - end_path_s) < dist) {
    decceleration = 0.4;
    set_speed_limit(vehicle_speed*2.24); // convert mps to mph
  } else {
    set_speed_limit(speed_limit);
    decceleration = 0.224;
  }
}