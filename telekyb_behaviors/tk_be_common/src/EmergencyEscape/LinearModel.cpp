#include "tk_be_common/EmergencyEscape/LinearModel.hpp"
//using namespace Supervisor;

/*
* Constructor, takes in a world value and constructs its own world based on it
* TODO: Should be changed to take in a measurement value (which it can then convert internally)
*/
LinearModel::LinearModel(const vector<float> &w0, const float &tspec)
{
	w = w0;
	ts = tspec;
}

/*
* Destructor
*/
LinearModel::~LinearModel()
{
	/* Nothing to see here */
}

/*
* Update the World pos and vel of the controller
*/
void LinearModel::PerformAction(const vector<float> &a)
{
	for (int i = 0; i < a.size(); ++i)
	{
		w[i]   = UpdatePosition(w[i], w[i+3], a[i]);
		w[i+3] = UpdateVelocity(w[i+3], a[i]);
	}
}
/*
* Get the world status (in the obstacle reference frame)
*/
vector<float> LinearModel::getWorld(){
	return w;
}

/*
* Get the measurement status
*/
Measure LinearModel::getMeasure(){
	return WorldToMeasure(w);
}

/*
* Get the state
*/
State LinearModel::getState(){
	return MeasureToState(WorldToMeasure(w));
}

/*
Update the position of the UAV according to the kinematic equations
*/
float LinearModel::UpdatePosition(const float &p, const float &v, const float &a)
{
	return p + v * ts + 0.5 * a * pow(ts,2);
}

/*
Update the velocity of the UAV according to the kinematic equations
*/
float LinearModel::UpdateVelocity(const float &v, const float &a)
{
	return v + a * ts;
}

