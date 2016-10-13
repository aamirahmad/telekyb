
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tk_be_common/EmergencyEscape/Reward.hpp>
#include <tk_be_common/EmergencyEscape/LinearModel.hpp>
#include <tk_be_common/EmergencyEscape/state.hpp>

#include <ecl/linear_algebra.hpp>
#include <vector>
#include <Eigen/Sparse>

using namespace ecl::linear_algebra;
using namespace std;
using namespace Eigen;

class Controller{
	float ts;
	double var;

	int ndstates, ndactions;

	MatrixXf thetamat;
	MatrixXf actionmat, statemat;

	vector<vector<float> > actionsequence;
	vector<float> inability_del, inability_acc, complexity;

	Reward r;
	
public:
	/* Constructor */
	Controller();

	/* Destructor */
	~Controller();
	
	void Initialize(const MatrixXf &tm, const MatrixXf &am, const MatrixXf &sm,  const MatrixXf &lim);

	vector<float> SelectAction(const State &s);
	void PredictIdealPath(const vector<float> &w);
	float DetermineComplexity(const int &i);
	float LimitCorrector(const vector<float> &w, const State &s, const vector<float> &delay);
	float Scaler(const vector<float> &w);
	void setAccelerationInability(const vector<float> &accinability);

private:

	float BasisFunction(const int &i, float sar[3]);
	VectorXf GetBFVector(const State &s);

	int argmax(const VectorXf &Q);
	template <typename MyVecType> MyVecType diff(const MyVecType &v);

};

#endif