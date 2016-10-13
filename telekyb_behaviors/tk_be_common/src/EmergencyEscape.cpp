/*
 * EmergencyEscape.cpp
 *
 *  Created on: Apr 23, 2015
 *      Author: mcoppola
 */

#include "EmergencyEscape.hpp"

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::EmergencyEscape, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

EmergencyEscape::EmergencyEscape()
	: Behavior("tk_be_common/EmergencyEscape", BehaviorType::Air),
	//w(6),
	wobst(6),
	acc(3),
	t(5.0),
	tstep(0.0)
{

}

void EmergencyEscape::initialize()
{
	ROS_INFO("Initialized EmergencyEscape Behavior.");
	/* Load all matrices from the txt files when node is initialized */
	//SparseMatrix<float> thetamatrix;
	MatrixXf thetamatrix;	
	MatrixXf actionspace,statespace,rewardlims;
	std::string path = ros::package::getPath("tk_be_common");

	actionspace = readMatrix((path+"/src/EmergencyEscape/actionspace.txt").c_str());
	statespace = readMatrix((path+"/src/EmergencyEscape/statespace.txt").c_str());
	thetamatrix = readMatrix((path+"/src/EmergencyEscape/thetamatrix.txt").c_str());
	rewardlims = readMatrix((path+"/src/EmergencyEscape/rewardlims.txt").c_str());

	//load controller and reward function
	controller.Initialize(thetamatrix, actionspace, statespace, rewardlims);

	sub1 = mainNodeHandle.subscribe("/synth_danger", 
		10,
		&Sread::SreadCB,
		&reader);

	sub2 = mainNodeHandle.subscribe("/sensor_danger", 
		10,
		&TKreader::TKreadCB,
		&reader2);

}

void EmergencyEscape::destroy()
{

}

bool EmergencyEscape::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	currentV = currentState.linVelocity;
	currentV_a = currentV;

	GetAvoidanceState();

	// Get the obstacle reference frame data
	if(GetAvoidanceData())
	{
		// This should be compared to the obstacle velocity
		ih = (atan2(wobst[4],wobst[3])-atan2(-wobst[1],-wobst[0]));
		if (ih > M_PI)
	    	ih = ih - 2*M_PI;
	}
	else 
	{
		ih = 0.0;
	}

	yawAngle = currentState.getEulerRPY()(2);
	begin = ros::Time::now();
	oldtstep = begin;
	counter = 1;
	return true;

	//t(0);

}

void EmergencyEscape::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void EmergencyEscape::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void EmergencyEscape::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void EmergencyEscape::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	/* same as active*/
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void EmergencyEscape::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	cout << endl;

	float rate = 0.2;

	// Reset every so often
	if (t.toSec() >= rate)
	{
		currentV = currentState.linVelocity; // update velocity info with real value
		currentV_a = currentV;
		
		accinput = getDesiredAcceleration(currentState);
		velinput = currentV + accinput * rate;
		begin = ros::Time::now();

		//
		cout << "\nNow at " << t.toSec() << "sec : " << currentV(0)<< " \t " << currentV(1) <<" \t " << currentV(2);		
	}
		t = ros::Time::now() - begin;



	/*
	t = ros::Time::now() - begin;
	n = t.toSec()/counter;

	//if ( counter % 5 == 0)
	{
		currentV = currentState.linVelocity; // update velocity info with real value
		currentV_a = currentV;

		//
		cout << "\nNow at " << t.toSec() << "sec : " << currentV(0)<< " \t " << currentV(1) <<" \t " << currentV(2);		
	}

	accinput = getDesiredAcceleration(currentState);
	currentV_a = currentV_a + accinput * n;
	velinput = currentV_a + accinput * n;
	*/

	//tstep = ros::Time::now();

//	dstep = tstep - oldtstep; // duration of one step = current time - previous current time
//	cout << endl << t.toSec() << "\t" << dstep.toSec() ;
	cout << "\nVel in.: " << velinput(0) << "\t" << velinput(1) << "\t" << velinput(2);

	//generatedTrajInput.setAcceleration(accinput);
	generatedTrajInput.setVelocity(velinput);
	generatedTrajInput.setYawAngle(yawAngle);

//	oldtstep = tstep;
	counter++;
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void EmergencyEscape::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	/* Set velocity to 0 */
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool EmergencyEscape::isValid(const TKState& currentState) const
{

}

bool EmergencyEscape::GetAvoidanceState()
{
	sobst = reader.state;

	if (reader.success = true)
		return true;
	else 
		return false;
}

bool EmergencyEscape::GetAvoidanceData()
{
	wobst = reader2.w;

	if (reader2.success = true)
		return true;
	else 
		return false;
}

Acceleration3D EmergencyEscape::getDesiredAcceleration(const TKState& currentState)
{
	Acceleration3D accin;
	State s;
	float pm;

	// Get state relative to obstacle	
	if(GetAvoidanceState())
	{
		pm = controller.Scaler(wobst);
		s.dch = sobst.dch/pm;
		s.dcv = sobst.dcv/pm;
		s.tc  = sobst.tc/pm;
		acc = controller.SelectAction(s);
	}
	
	else
	{ // just so we don't screw things up by accident if the topic subscriber is messing up
		acc[0] = 0.0; 	acc[1] = 0.0;	acc[2] = 0.0;
	}

	// Heading angle
	ang = atan2(currentState.linVelocity(1),currentState.linVelocity(0));

	// Clockwise rotation around z_earth
	// The actions are applied in the direction of the current velocity (top view)
	sy = sin(ang);
	cy = cos(ang);

	//sy = sin( ang - ih);
	//cy = cos( ang - ih);

	rot(0,0) = cy;
	rot(0,1) = sy;
	rot(0,2) = 0.0;

	rot(1,0) = -sy;
	rot(1,1) = cy;
	rot(1,2) = 0.0;

	rot(2,0) = 0.0;
	rot(2,1) = 0.0;
	rot(2,2) = 1.0;

	// Transform to required format
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			accin(i) += acc[j] * rot(j,i);
		}
	}
	
	//cout << "\nState:   " << sobst.dch << "\t\t" << sobst.dcv << "\t\t" << sobst.tc;
	//cout << "\nHeading: " << ang*180/M_PI << "\t\t ih : " << ih*180/M_PI;
	cout << "\nAccSel:  " << acc[0] << "\t\t" << acc[1] << "\t\t" << acc[2];
	cout << "\nAccAdj.: " << accin(0) << "\t" << accin(1) << "\t" << accin(2);

	return accin;
}

} /* namespace telekyb_behavior */

