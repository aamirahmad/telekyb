#include "tk_be_common/EmergencyEscape/Reward.hpp"

using namespace std;
//using namespace Supervisor;

/*
* Constructor:
* The constructor sets the limits as specified by the user
*/
Reward::Reward(){}

/*
* Destructor
*/
Reward::~Reward(){}

/* set the limits */
void Reward::setLimits(const float &lf, const float &ls)
{
	lim_fail 	= lf; 
	lim_success = ls;
}

/* get functions
* The variables in the class are protected,
* so here are some public functions to extract them
*/
float Reward::getlf(){ return lim_fail; }
float Reward::getls(){ return lim_success; }

/*
Returns the reward given a specific input state
*/
float Reward::determineReward(const State &s){

	reward.dch = Reward_dch(s.dch);
	reward.dcv = Reward_dcv(s.dcv);
	reward.tc  = Reward_tc(s.tc);
	
	return reward.dch + reward.dcv + reward.tc;
}

/* Determines the flag for success(1) or warning/failure (0) as a bool 
*  It uses an input limit to make the comparison */
bool Reward::determineFlag(const State &s, const float &lim)
{
	if (determineReward(s) > lim)
		return true;
	else
		return false;
}

/* Determines the flag for success(1) or warning/failure (0) as a bool 
*  It uses the updated lim_success to make the comparison */
bool Reward::determineFlag(const State &s)
{
	if (determineReward(s) > lim_success)
		return true;	
	else
		return false;
}

/*
Function to get the "inability" vector
*/
vector<float> Reward::GetInability(const State &s, const vector<float> &del)
{
	
	vector<float> inability(3);

	float ad = lim_success - (Reward_dch(s.dch) + Reward_dcv(s.dcv));
	
	/* This depends on the reward function used !!!
	* This part of the code could be changed with a solver function
	in the future, so that it does not need to be changed
	*/
	float tc_crossover = sqrt(ad);

	/* Now use the result */
	if (tc_crossover > 0.0){
		for (int i = 0; i < 3; ++i)
			inability[i] = Reward_tc(tc_crossover+del[i]) - Reward_tc(tc_crossover);
	}

	else {
		inability[0] = 0.0;
		inability[1] = 0.0;
		inability[2] = 0.0;
	}

	return inability;

}


/*
* Reward for d_ch:
* dc_h is defined symmetrically so, although it makes a difference
* in the action selection, the value is the same whether
* we are on the left or the right of the obstacle,
* so we just use the quadratic function all the time
*/
float Reward::Reward_dch(const float &x){
	return pow(x,2);
}

/*
* Reward for d_cv:
* d_cv is defined positive downwards.
* So if it is positive we are in the zone where we hit the body.
* If it is negative we are above the head and we can use the 
* quadratic function
*/
float Reward::Reward_dcv(const float &x){
	if (x > 0.0)
		return 0.0;
	else
		return pow(x,2);
}

/*
* Reward for t_c:
* If t_c is smaller than zero, we are happy because we are 
* moving away from the obstacle.
* Otherwise, we again use the quadratic function
*/
float Reward::Reward_tc(const float &x){
	if (x < 0.0)
		return 100.0; // Any large enough number will do
	else
		return pow(x,2);
}