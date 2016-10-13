/*
 * PositionControl.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef POSITIONCONTROL_HPP_
#define POSITIONCONTROL_HPP_

#include <ros/console.h>    
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

#include <tk_ctrlalgo/PosCtrlDefines.hpp>
#include <telekyb_base/Messages.hpp>

namespace TELEKYB_NAMESPACE {

class PositionControlOptions : public OptionContainer {
public:
	Option<double>* tPropGain;
	Option<double>* tXPropGain;
	Option<double>* tYPropGain;
	Option<double>* tDerivGain;
	Option<double>* tXDerivGain;
	Option<double>* tYDerivGain;
	Option<double>* tIntegGain;
	Option<double>* tXIntegGain;
	Option<double>* tYIntegGain;
	
	Option<double>* tZIntegGain;
	
	Option<double>* tIntegVelGain;

	Option<double>* tDerGainVelMode;

	Option<double>* tZPropGain;
	Option<double>* tZDerivGain;

	Option<double>* tSatIntTerm;
	Option<double>* tSatIntVelTerm;
	
	Option<double>* tMinThrust;

	Option<double>* tMaxRollSin;
	Option<double>* tMaxPitchSin;
	//XXX: this is not good practice since it might accidentally report accs of the wrong vehicle!
//	Option<bool>* tPubCommandedAcc;
	
	//Option<double>* tMaxVelInVelMode;

	Option<double>* tGravity;
	
	PositionControlOptions();
};

class PositionControl {
private:
	PositionControlOptions options;

	//double mass;
	double xIntErr, yIntErr, zIntErr, xIntVelErr, yIntVelErr, zIntVelErr, yawRateErr;
	
	// temp variables
	FILE * fileTemp1;
	FILE * fileTemp2;
// 	double firstTime;
	double time;
	
	//double gravity;

	Timer integTimer;
	//XXX: this is not good practice since it might accidentally report accs of the wrong vehicle!
//	ros::Publisher accPublisher;
	ros::NodeHandle mainNodehandle;
    tf::TransformListener tfListener;

public:
	// gravity is provided by
	PositionControl();
	virtual ~PositionControl();

	void run(const TKTrajectory& input, const TKState& currentState, const double mass, PosCtrlOutput& output);
	//void setMass(double mass_);
};

}

#endif /* POSITIONCONTROL_HPP_ */
