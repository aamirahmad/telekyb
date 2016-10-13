#ifndef LINEARMODEL_H
#define LINEARMODEL_H

#include <vector>
#include "tk_be_common/EmergencyEscape/state.hpp"

using namespace std;

class LinearModel{
	vector<float> w;
	float ts;
	
public:

	/* Constructor */
	LinearModel(const vector<float> &w0, const float &tspec);

	/* Destructor */
	~LinearModel();

	/* Perfom the action */
	void PerformAction(const vector<float> &a);

	/* Get out some useful data */
	vector<float> getWorld();
	Measure getMeasure();
	State getState();

protected:
	/* Update the position */
	float UpdatePosition(const float &p, const float &v, const float &a);

	/* Update the velocity */
	float UpdateVelocity(const float &v, const float &a);
};

#endif