

#ifndef VARIANCE_ESTIMATOR_HPP_
#define VARIANCE_ESTIMATOR_HPP_

class VarianceEstimator
{
	double n;
	double delta;
	double mean;
	double M2;
	
public:
	VarianceEstimator(double initialVariance = 0.2);
	
	double step(double value);
};

#endif 

