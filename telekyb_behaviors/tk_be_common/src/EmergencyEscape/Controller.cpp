#include "tk_be_common/EmergencyEscape/Controller.hpp"

using namespace std;
using namespace ecl::linear_algebra;
using namespace Eigen;

#define spread 20 // spread limit for matrix multiplication around fuzzy center (improves speed)
#define phorizon 5 // prediction horizon

/*
* Constructor
*/
Controller::Controller():
inability_del(3), inability_acc(3), complexity(3)
{
	// Data for the controller
	ts = 0.2;
	var = 0.01;

	// Initialize these, they can be reset later
	inability_acc[0] = 0;
	inability_acc[1] = 0;
	inability_acc[2] = 0;

	// Resize action sequence matrix to fit expectations
	actionsequence.resize(3); //columns
    for(int j = 0; j < 3; j++)
    	actionsequence[j].resize(phorizon); //rows

}

/*	
* Destructor
*/
Controller::~Controller(){
	/* Nothing to see here */
}

//void Controller::Initialize(const SparseMatrix<float> &tm, const MatrixXf &am, const MatrixXf &sm, const MatrixXf &lim)
void Controller::Initialize(const MatrixXf &tm, const MatrixXf &am, const MatrixXf &sm, const MatrixXf &lim)
{	

	// Some useful data on the size of the matrices
	ndstates = sm.rows(); 
	ndactions = am.rows();

	// Matrix assignment within controller
	thetamat = tm;
	actionmat = am;
	statemat = sm;

	// Set the limits of the reward scheme
    r.setLimits(lim(0),lim(1));
}

/*
* Select the best course of action in state s
*/
vector<float> Controller::SelectAction(const State &s)
{
	vector<float> a(3);
	int aindex, bfcenter, bf_llim;
	VectorXf bf;
	
	// Calculate the Basis Function Vector for the given state
	bf = GetBFVector(s);
	bf.maxCoeff(&bfcenter);

	bf_llim = bfcenter - spread;

	if (bf_llim < 0)
		bf_llim = 0;
	if (bf_llim >= ndstates)
		bf_llim = ndstates - (spread*2) - 1;

	//cout << "vec\n";
	VectorXf bfc = bf.segment<spread*2>(bf_llim);
	//cout << "mat\n";
	MatrixXf thetamult = thetamat.block(bf_llim,0,spread*2,ndactions-1);

	VectorXf slowmult; 
	slowmult.noalias() = bfc.transpose() * thetamult;
		
	slowmult.maxCoeff(&aindex);

	// Get the corresponding action on each column
	for (int i = 0; i < a.size(); ++i){
		if (i < 2)
			a[i] = actionmat(aindex,i);
		else 
			a[i] = actionmat(aindex,i);
	}

	return a;
}

/*
* PredictIdealPath:
* This takes care of looking ahead and predicting the ideal action sequece vector
*/
void Controller::PredictIdealPath(const vector<float> &w)
{
	// Action vector
	vector<float> action(3);
	
 	// Create a linear model
	LinearModel model(w, ts);

	// Run a simulation for a set number of times on a linear model
	for (int i = 0; i < phorizon; ++i)
	{
		// Select and store the perfect action
		action = SelectAction(model.getState());
		
		// Perform the given action on a model
		model.PerformAction(action);

		// Store the action
		actionsequence.push_back(action);
	}
}

/*
* Returns the complexity for a given action sequence in a vector
*/
float Controller::DetermineComplexity(const int &i)
{
	float c = 0;
	vector<float> a(phorizon-1);
	vector<float> actseq(phorizon);

	for (int j = 0; j < phorizon; ++j)
		actseq[j] = actionsequence[i][j];
		
	a = diff(actseq);

	// Calculate Difference in future actions
	for (int i = 0; i < a.size() ; ++i)
		c += a[i];

	return c;
}

/* This is the main subroutine that provides the new limit based on the current state */
float Controller::LimitCorrector(const vector<float> &w, const State &s, const vector<float> &delay)
{
	float p, pm, b, t, tm;

	// Start fresh, since the next iteration may be compleately different
	t = 0;
	tm = 0;
	b = 0;
	actionsequence.clear();

	// Predict the path over the prediction horizon
	PredictIdealPath(w);

	// Determine the inability of the controller based on the initial conditions
	inability_del = r.GetInability(s, delay);

	for (int i = 0; i < 3; ++i)
	{
		complexity[i] = DetermineComplexity(i);
		t  += inability_del[i]*complexity[i];
		tm += inability_acc[i]*complexity[i];
		b  += complexity[i];
	}
	
	// Return the updated success limit
	p =  t/b;
	pm = tm/b;

	// Exclude the case for complexity = 0
	if (!isfinite(p))
		p = 0;

	if (!isfinite(pm))
		pm = 0;

	// Return the updated limit
	return r.getls()*(1.0+pm) + p;
}

float Controller::Scaler(const vector<float> &w)
{
	float pm, b, tm;

	// Start fresh, since the next iteration may be compleately different
	tm = 0;
	b = 0;
	actionsequence.clear();

	// Predict the path over the prediction horizon
	PredictIdealPath(w);

	for (int i = 0; i < 3; ++i)
	{
		complexity[i] = DetermineComplexity(i);
		tm += inability_acc[i]*complexity[i];
		b  += complexity[i];
	}
	
	// Return the updated success limit
	pm = tm/b;

	if (!isfinite(pm))
		pm = 0;

	// Return the updated limit
	return 1.0 + pm;
}

/*
* The basis function as used to get the Basis Function vector
*/
float Controller::BasisFunction(const int &i, float sar[3])
{
	double t = 0;

	for (int j = 0; j < 3; ++j)
		t = t + (pow(sar[j]-statemat(i,j),2) / (2*var));
	
	return exp(-t);
}

/*
* Get the Basis Function vector
*/
VectorXf Controller::GetBFVector(const State &s)
{	
	VectorXf bf(ndstates);

	float sar[3];

	sar[0] = s.dch;
	sar[1] = s.dcv;
	sar[2] = s.tc;

	for (int i = 0; i < ndstates; i++)
		bf(i) = BasisFunction(i, sar);

	return bf;	
}


/*
* argmax:
* Returns index (action) of largest entry in Q array, breaking ties randomly
* Based on the example function by Sutton and Barto for the mountain car example cpp file
*/
int Controller::argmax(const VectorXf &Q)
{ 
	int   best_action = 0;
	float best_value  = Q(0);
   	int   num_ties    = 1;  // actually the number of ties plus 1
    
	for ( int a = 0; a <= ndactions; a++ ) // go through all elements of vector Q
    {
    	float value = Q(a); 

    	/*If the value is larger than the best_value*/
        if (value >= best_value) 
        {
        	/* If it's actually larger, reassign.
        	* Store value and position. That's pretty straightforward */
            if (value > best_value)
            {
            	best_value = value;
            	best_action = a;
            }
	               
	        /* Otherwise, break the tie */
	        else
	        {
	           	num_ties++;			// Count the number of ties
	           	/*
	            * 'rand() % num_ties' returns a random integer between 0 and num_ties
				* So then we randomly decide to assign it based on its outcome
				*  The probability of reassignment decreases with more ties.
				*  It's pretty unlikely to get ties anyway.
				*/
	            if (0 == rand() % num_ties)
				{
					best_value = value;
					best_action = a;
				}
			}
		}
	}

    return best_action;	// Return action index
}

/*
* Returns a vector of the same type showing the difference between elements of the input vector
*/
template <typename MyVecType>
MyVecType Controller::diff(const MyVecType &v)
{
	MyVecType A(v.size());

	for (int i = 0; i < (phorizon - 1) ; i++)
		A[i] =  abs(v[i+1] - v[i]);

	return A;
}


void Controller::setAccelerationInability(const vector<float> &accinability)
{
	for (int i = 0; i < 3; ++i)
		inability_acc[i] = accinability[i];
}