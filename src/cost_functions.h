/*
* cost_functions.h
* A library that will hold all the necessary cost functions for selecting the best state.
*/

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

#define LANE_WIDTH 4.0 // m
#define SPEED_LIMIT 50.0 // mph
#define SAFE_DISTANCE 59 // m

class Cost
{
public:
	// constructor
	Cost();
	
	// destructor
	~Cost();

	// cost function for observing speed limit
	double speed_limit_cost(double ego_v);

	// cost function for avoiding collision along heading of the car
	double collision_cost(double ego_s, double other_s);

	//cost function for time to collision
	double time_to_collide(double ego_s, double other_s, double ego_v, double other_v);


private:

};

Cost::Cost()
{
	// nothing special to be initialized in constructor, yet!
}

Cost::~Cost()
{
	// nothing special to be added in destructor, yet!
}

double Cost::speed_limit_cost(double ego_v) 
{
	// penalize if the ego car is below or above the speed limit
	double delta_vel = (SPEED_LIMIT - ego_v) / SPEED_LIMIT;
	return powf(delta_vel, 2);
}

double Cost::collision_cost(double ego_s, double other_s)
{
	// penalize if the ego car is too close to another car
	double delta_s = other_s - ego_s;

	if (fabs(delta_s)  < 1.0 )
	{
		return 1;
	}
	else
	{
		double delta_s_not = SAFE_DISTANCE;
		double diff = (1 / fabs(delta_s) - 1 / delta_s_not);
		return 1 / (1 + exp(-SAFE_DISTANCE*diff));
	}
}

double Cost::time_to_collide(double ego_s, double other_s, double ego_v, double other_v)
{
	// ego_v will be in mph. Convert to mps
	ego_v = ego_v * 0.44704;
	double relative_vel = fabs(ego_v - other_v);
	double distance = fabs(ego_s - other_s);
	double t = distance / relative_vel;
	return exp(-0.009*t*t);
}
