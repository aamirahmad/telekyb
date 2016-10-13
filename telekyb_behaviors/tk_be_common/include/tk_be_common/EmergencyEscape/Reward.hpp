#ifndef REWARD_H
#define REWARD_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <tk_be_common/EmergencyEscape/state.hpp>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Sparse>

using namespace std;

class Reward{
	friend class Controller;

	State reward;

public:

	float lim_fail;
	float lim_success;

	/* Constructor */
	Reward();

	/* Destructor */
	~Reward();

	/* Set up the limits, initialize */
	void setLimits(const float &lf, const float &ls);
	
	float getlf();
	float getls();

	/* Calculate the reward based on a state */
	float determineReward(const State &state);

	/* Function to determine success(1) or failure/warning(0) */
	bool determineFlag(const State &s, const float &lim);
	bool determineFlag(const State &s);  // some overloading is required...
	
	vector<float> GetInability(const State &s, const vector<float> &del);

private:
	/* The individual reward functions */
	float Reward_dch(const float &dch);
	float Reward_dcv(const float &dcv);
	float Reward_tc(const float &tc);


};

#endif
