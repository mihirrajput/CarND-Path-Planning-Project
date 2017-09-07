/*
This library realizes the following states:
Keep Lane
Left Change Lane
Right Change Lane
*/

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

class Realize
{
public:
	Realize();
	~Realize();

	// find car in my lane
	int car_in_my_lane(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size);

	// realize keep lane state
	double realize_keep_lane(vector<vector<double>> sensor_fusion, int min_car_id, double ref_vel, double car_s, int prev_size);

	// find car in left lane to me
	vector<int> car_in_left_lane(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size);

	// realize left lane change state
	int realize_left_lane_change(int lane);

	// find car in right lane to me
	vector<int> car_in_right_lane(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size);

	// realize left lane change state
	int realize_right_lane_change(int lane);

private:

	bool too_close = false;

	vector<int> left_car_id;

};

Realize::Realize()
{
}

Realize::~Realize()
{
}

int Realize::car_in_my_lane(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size)
{
	double check_speed = 99.0; // mps
	double min_car_s_diff = 999.9; // m
	int min_car_id = 99;

	// find ref_vel to use for ego car
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		// is car in my lane?
		double d = sensor_fusion[i][6];
		if (d > lane * 4 && d < lane * 4 + 4)
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			check_speed = sqrt(vx*vx + vy*vy); // in mps
			double check_car_s = sensor_fusion[i][5];
			// project s value of the car in my lane to end of the previous path point
			check_car_s += prev_size*0.02*check_speed;

			if ((check_car_s > car_s) && (check_car_s - car_s < 30))
			{
				too_close = true;
				// figure out the closest car to me!
				if (check_car_s - car_s < min_car_s_diff)
				{
					min_car_s_diff = check_car_s - car_s;
					min_car_id = i; //index not id...yet
				}
				
			}
		}
	}
	return min_car_id;
}

double Realize::realize_keep_lane(vector<vector<double>> sensor_fusion, int min_car_id, double ref_vel, double car_s, int prev_size)
{
	if (too_close == true && min_car_id != 99)
	{
		double vx = sensor_fusion[min_car_id][3];
		double vy = sensor_fusion[min_car_id][4];
		double check_speed = sqrt(vx*vx + vy*vy); // in mps
		double check_car_s = sensor_fusion[min_car_id][5];
		if (ref_vel > check_speed*2.23) 
		{
			ref_vel -= 0.41; //decelerate when the car is too close
		}
	}

	else if (too_close == false && ref_vel < 49.5)
	{
		ref_vel += 0.41;
	}

	too_close = false; // can be deleted later
	return ref_vel;
}

vector<int> Realize::car_in_left_lane(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size)
{
	double check_speed = 99.0; // mps
	vector<int> left_car_id;
	left_car_id.clear();

	// find ref_vel to use for ego car
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		// is car in my lane?
		double d = sensor_fusion[i][6];
		if (d >(lane - 1) * 4 && d < (lane - 1) * 4 + 4)
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			check_speed = sqrt(vx*vx + vy*vy); // in mps
			double check_car_s = sensor_fusion[i][5];
			// project s value of the car in my lane to end of the previous path point
			check_car_s += prev_size*0.02*check_speed;
			if (check_car_s - car_s < 59 || car_s - check_car_s < 15)
			{
				left_car_id.push_back(i);
			}
		}
	}

	if (left_car_id.size() == 0)
	{
		left_car_id.push_back(99);
	}

	return left_car_id;
}

int Realize::realize_left_lane_change(int lane)
{
	return lane-1;
}

vector<int> Realize::car_in_right_lane(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size)
{
	double check_speed = 99.0; // mps
	vector<int> right_car_id;
	right_car_id.clear();

	// find ref_vel to use for ego car
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		// is car in my lane?
		double d = sensor_fusion[i][6];
		if (d >(lane + 1) * 4 && d < (lane + 1) * 4 + 4)
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			check_speed = sqrt(vx*vx + vy*vy); // in mps
			double check_car_s = sensor_fusion[i][5];
			// project s value of the car in my lane to end of the previous path point
			check_car_s += prev_size*0.02*check_speed;
			if (check_car_s - car_s < 59 || car_s - check_car_s < 15)
			{
				right_car_id.push_back(i);
			}
		}
	}

	if (right_car_id.size() == 0)
	{
		right_car_id.push_back(99);
	}

	return right_car_id;
}

int Realize::realize_right_lane_change(int lane)
{
	return lane + 1;
}