
#include <telekyb_base/Estimators/VarianceEstimator.hpp>


VarianceEstimator::VarianceEstimator(double initialVariance)
{
	n = 1;
	mean = 0;
	M2 = initialVariance;
}

double VarianceEstimator::step(double value)
{
	n = n + 1;
        delta = value - mean;
        mean = mean + delta/n;
        M2 = M2 + delta*(value - mean);
	
	return M2/(n - 1);
	
}

