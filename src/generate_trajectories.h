/*
* generate_trajectories.h
* A library that will genreate trajectory based on the state: CS, KL, PLCL, RLCL, LCL, RCL.
*/

#include <math.h>
#include <iostream>
#include <vector>
#include "spline.h"
#include "helper.h"

using namespace std;

Help h;

// define a spline
tk::spline s;


class Trajectory
{
public:
	// constructor
	Trajectory();

	// destructor
	~Trajectory();

	// creates a trajectory for the state ego car is supposed to be in
	vector<vector<double>> create_trajectory(double car_x, double car_y, double car_yaw, double car_s, int lane, double ref_vel, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

private:

};

Trajectory::Trajectory()
{
	// nothing special to be initialized in constructor, yet!
}

Trajectory::~Trajectory()
{
	// nothing special to be initialized in constructor, yet!
}

// creates a trajectory for the state ego car is supposed to be in
// arguments: ego car state and preveious points
vector<vector<double>> Trajectory::create_trajectory(double car_x, double car_y, double car_yaw, double car_s, int lane, double ref_vel, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
	// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	// Later we will interpolate these waypoints with a spline and fill it in with more points that control speed
	vector<double> ptsx;
	vector<double> ptsy;

	// reference x,y and yaw states
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = h.deg2rad(car_yaw);

	int prev_size = previous_path_x.size();

	// if previous size is close to empty, use car as starting reference
	if (prev_size < 2)
	{
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);
		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	}
	// use previous path's end as starting reference
	else
	{
		// Redefine reference state as previous path end point
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		//use two points that make the path tangent to previous path's end point
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	// In Frenet add evenly 30 m spaced points ahead of the starting reference
	vector<double> next_wp0 = h.getXY(car_s + 30, 2 + lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = h.getXY(car_s + 60, 2 + lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = h.getXY(car_s + 90, 2 + lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	// Transform to local ego car frame
	for (int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}

	/*// define a spline
	tk::spline s;*/

	// pass (x,y) points to it
	s.set_points(ptsx, ptsy);

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	// Start with all previous path points from last time
	for (int i = 0; i < previous_path_x.size(); i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// Calculate how to break up spline points so that we travel at our desired reference velocity
	double target_x = 30;
	double target_y = s(target_x);
	double target_dist = h.distance(0.0, 0.0, target_x, target_y);

	double x_addon = 0;

	for (int i = 1; i < 50 - previous_path_x.size(); i++)
	{
		double N = target_dist / (0.02 * ref_vel * 0.447);
		double x_point = x_addon + target_dist / N;
		double y_point = s(x_point);
		x_addon = x_point;

		double x_ref = x_point;
		double y_ref = y_point;
		// transform back to global frame
		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	vector<vector<double>> next_xy_vals;
	next_xy_vals.push_back(next_x_vals);
	next_xy_vals.push_back(next_y_vals);
	return next_xy_vals;
}

