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

std::vector<double> Planner::jmt_coefficients(std::vector<double> start, std::vector<double> end, double T) {
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    std::vector<double> alphas = {0, 0, 0, 0, 0, 0};
    
    alphas[0] = start[0];
    alphas[1] = start[1];
    alphas[2] = 0.5 * start[2];
    
    Eigen::VectorXd b(3);
    Eigen::MatrixXd A(3, 3);
    b << end[0] - (start[0] + start[1]*T + 1.0/2*start[2]*T*T),
         end[1] - (start[1] + start[2]*T),
         end[2] - start[2];
         
    A << pow(T, 3), pow(T, 4), pow(T, 5),
         3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
         6*T, 12*pow(T, 2), 20*pow(T, 3);
    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    
    alphas[3] = x[0];
    alphas[4] = x[1];
    alphas[5] = x[2];
    return alphas;
}

double Planner::jmt(double t, std::vector<double> alphas) {
  double result = 0.0;
  for (int i = 0; i < alphas.size(); i++) {
    result += alphas[i] * pow(t, i);
  }
  return result;
}

std::vector<std::vector<double>> Planner::generate_jm_trajectory(double goal) {
  int prev_size = previous_path_x.size();
  // Global car coordinates in reference frame
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw); 
  double v0 = car_speed;

  if (prev_size > 2) {
    // redefine reference points
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    v0 = sqrt(pow(ref_x - ref_x_prev, 2) + pow(ref_x - ref_x_prev, 2))/delta_t;
  }

  std::vector<double> car_sd_ref = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

  if (target_speed < speed_limit) {
      target_speed += acceleration;
    // cout << "New reference speed: " << target_speed << endl;
  } else {
    target_speed -= decceleration;
  }

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  if (goal_lane_n != lane_n) {
    lane_n = goal_lane_n;
  }
  
  double T = 0.0;
  double v1 = 0.0;
  if (v0 >= speed_limit/2.24) {
    v0 = speed_limit/2.24;
    T = (-v0 + sqrt(v0*v0 + 2*acceleration*goal))/acceleration; 
    v1 = v0;
  } else {
    T = (-v0 + sqrt(v0*v0 + 2*acceleration*goal))/acceleration; 
    v1 = v0 + acceleration * T;
  }

  double goal_d = 2+4*goal_lane_n - car_sd_ref[1];
  double goal_alpha = asin(goal_d/goal);
  double goal_s = goal*cos(goal_alpha);  

  cout << "Problem parameters: " << endl;
  cout << "T: " << T << endl;
  cout << "goal_s: " << goal_s << endl;
  cout << "goal_d: " << goal_d << endl;
  cout << "v0: " << v0 << endl;

  std::vector<double> start_s = {0.0, v0, 0.0};
  std::vector<double> end_s = {goal_s, v1, 0.0};
  std::vector<double> s_coeffs = jmt_coefficients(start_s, end_s, T);

  /*
  std::vector<double> start_d = {0.0, 0.0, 0.0};
  std::vector<double> end_d = {goal_d, 0.0, 0.0};
  std::vector<double> d_coeffs = jmt_coefficients(start_d, end_d, T);
 
  cout << "s coefficents: " << endl;
  for(int i = 0; i < s_coeffs.size(); i++) {
    cout << s_coeffs[i] << endl;
  }
  
  cout << "d coefficents: " << endl;
  for(int i = 0; i < d_coeffs.size(); i++) {
    cout << d_coeffs[i] << endl;
  }
  
  cout << "Try some predictions: " << endl;
  double dt_next = delta_t;
  while (dt_next <= 1.0) {
    cout << "dt: " << dt_next << endl;
    cout << jmt(dt_next, d_coeffs) << endl;
    dt_next += delta_t;
  }*/

  for (int i = 0; i <= 50 - prev_size; i++) {

    double dt_next = (i+1)*delta_t;
    double s_next = jmt(dt_next, s_coeffs);
    double d_next = 0;
    cout << "s next: " << s_next << " d_next: " << d_next << endl;
    
    std::vector<double> xy_point = getXY(car_sd_ref[0] + s_next, car_sd_ref[1] + d_next, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    cout << "x next: " << xy_point[0] << " y next: " << xy_point[1] << endl;

    double x_point = xy_point[0];
    double y_point = xy_point[1];
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  #if DEBUG 
    cout << "x-points: " << endl;
    for(int i = 0; i < next_x_vals.size(); i++) {
      cout << next_x_vals[i] << endl;
    } 

    cout << "y-points: " << endl;
    for(int i = 0; i < next_y_vals.size(); i++) {
      cout << next_y_vals[i] << endl;
    }
  #endif

  return {next_x_vals, next_y_vals};
}

std::vector<std::vector<double>> Planner::generate_trajectory(double goal) {
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
    double N = target_dist/(0.02 * target_speed/2.24);

    for (int i = 0; i <= 50 - prev_size; i++) {
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

    #if DEBUG 
      cout << "x-points: " << endl;
      for(int i = 0; i < next_x_vals.size(); i++) {
        cout << next_x_vals[i] << endl;
      } 

      cout << "y-points: " << endl;
      for(int i = 0; i < next_y_vals.size(); i++) {
        cout << next_y_vals[i] << endl;
      }
    #endif

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
  //cout << "Cost calculation process, lane: " << lane << endl;
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
        //cout << "Lane change is dangerous" << endl;
        return 100;
      } else {
        //cout << "Lane change is possible. Calculating cost" << endl;
        // count it
        counter ++;
        // calc average speed
        avg_speed += 1.0/counter * (vehicle_speed - avg_speed);
        cost = (int)avg_speed;
      }
		}
	}
  //cout << "Resulting cost: " << cost << endl;
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
  //cout << "Keep distance routine" << endl;
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