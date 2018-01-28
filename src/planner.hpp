#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <math.h>
#include <vector>

using namespace std;

class PathPlanner {
public:
	const int LANE_WIDTH = 4;
	const double SPEED_LIMIT = 22.3;	// mph
	const double MAX_LANE_COST = 1000.0;
	const double FRONT_MAX_DIST = 100.0;
	const double BACK_MAX_DIST = 1000.0;
	const int MAX_A = 9; // maximum acceleration
	const double LANE_CHANGE_PENALTY = .2;

	double curr_lead_vehicle_speed;
	double curr_speed = 0;
	int curr_lane = 1;
	int curr_target_lane = 1;
	int prev_size = 0;


	vector<double> getClosestVehicle(double car_s, double front_dist,
		double front_speed, double back_dist, double back_speed,
		vector<vector<double> > vehicles);

	vector<double> computeLaneCost(double car_s,
		vector<vector<double> > vehicles, bool sameLane,
		const vector<vector<double> >& far_lane_vehicles = vector<vector<double> >());

	vector<double> computeStrategy(double car_s, double car_d,
		vector<vector<double> > sensor_fusion);

	int getLane(double d) { return (int)(d / LANE_WIDTH); }
}; 

#endif /* PLANNER_HPP */