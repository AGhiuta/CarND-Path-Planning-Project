#include "planner.hpp"

vector<double> PathPlanner::getClosestVehicle(double car_s, double front_dist,
	double front_speed, double back_dist, double back_speed, 
	vector<vector<double> > vehicles) {

	for(int i = 0; i < vehicles.size(); ++i) {
		double vx = vehicles[i][3];
		double vy = vehicles[i][4];

		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = vehicles[i][5];

		check_car_s += (double)prev_size * .02 * check_speed;

		if((check_car_s > car_s) && (check_car_s - car_s < front_dist)) {
			front_dist = max(1.0, check_car_s - car_s);

			// if the vehicle in the front is close,
			// slow down to its speed in order to keep a constant distance
			if(front_dist < 20.0) {
				front_speed = check_speed;
			}

			// if the vehicle in the front is too close,
			// slow down even further in order to put some distance
			// between the two cars
			if(front_dist < 10.0) {
				front_speed *= .5;
			}
		}

		// check the distance to the car behind on the target lane
		if((car_s >= check_car_s) && (car_s - check_car_s < 5.0)) {
			back_dist = max(1.0, car_s - check_car_s);
			back_speed = check_speed;
		}
	}

	// return the closest vehicles in the front and behind
	return {front_dist, front_speed, back_dist, back_speed};
}

vector<double> PathPlanner::computeLaneCost(double car_s,
	vector<vector<double> > vehicles, bool sameLane,
	const vector<vector<double> >& far_lane_vehicles) {

	vector<double> closest_vehicle = getClosestVehicle(car_s, FRONT_MAX_DIST,
		SPEED_LIMIT, BACK_MAX_DIST, SPEED_LIMIT, vehicles);

	// compute the cost of the lane as the inverse of the
	// distances to the closest vehicles in the front and behind
	// if the cost is computed for the current lane, then take
	// into account only the distance to the vehicle in the front
	double cost = 1.0 / closest_vehicle[0] + (1.0 - sameLane) / closest_vehicle[2];
	double target_speed = closest_vehicle[1];

	if(sameLane) {
		curr_lead_vehicle_speed = closest_vehicle[1];
	}

	// if current lane is either lane #0 or #2, 
	// and the furthest lane is less busy, check if it
	// is possible to prepare for that lane by 
	// getting on the middle lane first;
	// this is done in order to avoid future traffic jams
	if (far_lane_vehicles.size() > 0) {
		vector<double> far_lane_closest_vehicle = getClosestVehicle(car_s,
			FRONT_MAX_DIST, SPEED_LIMIT, BACK_MAX_DIST, SPEED_LIMIT, far_lane_vehicles);

		double far_lane_cost = 1.0 / far_lane_closest_vehicle[0] + 
			1.0 / far_lane_closest_vehicle[2];

		if(far_lane_cost < cost) {
			if(closest_vehicle[0] > 15) {
				if(closest_vehicle[2] > 5) {
					cost = far_lane_cost;
				}
			}
		}
	}

	return {cost, target_speed, closest_vehicle[0], closest_vehicle[2]};
}

vector<double> PathPlanner::computeStrategy(double car_s, double car_d,
	vector<vector<double> > sensor_fusion) {

	curr_lane = getLane(car_d);
	vector<vector<double> > left_lane_vehicles;
	vector<vector<double> > keep_lane_vehicles;
	vector<vector<double> > right_lane_vehicles;
	vector<vector<double> > far_left_lane_vehicles;
	vector<vector<double> > far_right_lane_vehicles;

	for(int i = 0; i < sensor_fusion.size(); ++i) {
		double d = sensor_fusion[i][6];
		int target_lane = getLane(d);

		if (target_lane == curr_lane) {
			// get the cars on the current lane
			keep_lane_vehicles.push_back(sensor_fusion[i]);
		} else if(target_lane == curr_lane - 1) {
			// get the cars on the lane to the left
			left_lane_vehicles.push_back(sensor_fusion[i]);
		} else if(target_lane == curr_lane + 1) {
			// get the cars on the lane to the right
			right_lane_vehicles.push_back(sensor_fusion[i]);
		} else if (target_lane == curr_lane - 2) {
			// get the cars on the lane to the far left
			// (if current lane is lane #2)
			far_left_lane_vehicles.push_back(sensor_fusion[i]);
		} else if (target_lane == curr_lane + 2) {
			// get the cars on the lane to the far right
			// (if current lane is lane #0)
			far_right_lane_vehicles.push_back(sensor_fusion[i]);
		}
	}

	vector<double> left_lane_cost = {MAX_LANE_COST, 0.0};
	vector<double> keep_lane_cost = {MAX_LANE_COST, 0.0};
	vector<double> right_lane_cost = {MAX_LANE_COST, 0.0};

	double target_cost = MAX_LANE_COST;
	double target_lane = curr_lane;
	double target_speed = curr_speed;
	double target_front_dist = FRONT_MAX_DIST;
	double target_back_dist = BACK_MAX_DIST;

	// compute the score of a left lane change, if that's possible
	if(curr_lane > 0) {
		left_lane_cost = computeLaneCost(car_s,
			left_lane_vehicles, false,
			far_left_lane_vehicles);
	}

	// compute the score of keeping the current lane
	keep_lane_cost = computeLaneCost(car_s,
			keep_lane_vehicles, true);

	// compute the score of a right lane change, if that's possible
	if(curr_lane < 2) {
		right_lane_cost = computeLaneCost(car_s,
			right_lane_vehicles, false,
			far_right_lane_vehicles);
	}

	// get the target lane with the minimum cost
	if (left_lane_cost[0] + LANE_CHANGE_PENALTY*fabs(left_lane_cost[0]) < target_cost) {
		target_cost = left_lane_cost[0] + LANE_CHANGE_PENALTY*fabs(left_lane_cost[0]);
		target_lane = curr_lane - 1;
		target_speed = left_lane_cost[1];
		target_front_dist = left_lane_cost[2];
		target_back_dist = left_lane_cost[3];
	}

	if(keep_lane_cost[0] < target_cost) {
		target_cost = keep_lane_cost[0];
		target_lane = curr_lane;
		target_speed = keep_lane_cost[1];
		target_front_dist = keep_lane_cost[2];
		target_back_dist = keep_lane_cost[3];
	}

	if(right_lane_cost[0] + LANE_CHANGE_PENALTY*fabs(right_lane_cost[0])< target_cost) {
		target_cost = right_lane_cost[0] + LANE_CHANGE_PENALTY*fabs(right_lane_cost[0]);
		target_lane = curr_lane + 1;
		target_speed = right_lane_cost[1];
		target_front_dist = right_lane_cost[2];
		target_back_dist = right_lane_cost[3];
	}

	/*
	return the target lane, target speed and distances to
	the closest vehicle in front, and the closest one behind
	*/
	return {target_lane, target_speed, target_front_dist, target_back_dist};
}