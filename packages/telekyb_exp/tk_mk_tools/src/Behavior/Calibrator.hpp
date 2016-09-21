/*
 * Calibrator.hpp
 *
 *  Created on: Feb 8, 2012
 *      Author: mriedel
 */

#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

//#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
//#include <sensor_msgs/Joy.h>

#include <telekyb_base/Time.hpp>

#include <telekyb_interface/MKInterface.hpp>
#include <boost/thread.hpp>

// to calculate median
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>

using namespace TELEKYB_NAMESPACE;
using namespace boost::accumulators;

namespace telekyb_behavior {

class Calibrator : public Behavior {
protected:
	// Option references
	Option<bool>* tDoMassEstimation;
	bool tDoMassEstimationSave;
	Option<double>* tPCIntegralGain;
	double tPCIntegralGainSave;

	// turn active conditions
	Option<double>* tMaxInitialVelocity;
	Option<double>* tMaxInitialYawError;

	Option<double>* tSettleTime;
	Option<int>* tValueRange;
	Option<int>* tCenterValueX;
	Option<int>* tCenterValueY;

	// Error Matrix
	Eigen::MatrixXd errorMatrix;

	// current Field
	Eigen::Vector2i currentField;
	// this is the actual field to test.
	//Eigen::Vector2d currentTestField;

	// 9 cases (0-9. mod -> row, div -> col);
	//int currentTestOffset;

	// Test Variables
	bool initialSetup;
	//double maxErrorTest;
	//double meanErrorTest;
	//Eigen::Vector2d meanErrorVector;
	//int meanNrSamples;

	// Position to hold
	Position3D calibrationPosition;

	// Timers
	Timer testTimer;
	Timer behaviorActiveTimer;

	Timer transitionTimer;

	// ROS
	ros::NodeHandle nodeHandle;

	// MKInterface to directly communicate with the QC.
	telekyb_interface::MKInterface* mkInterface;
	MKSingleValuePacket offsetRawAcc_X;
	MKSingleValuePacket offsetRawAcc_Y;

	bool valuesSet;
	bool setValueThreadDone;

	double jumpConstant;

	// Set Value Thread!
	boost::thread setValueThread;
	//boost::posix_time::time_duration timeout = boost::posix_time::milliseconds(500);

	accumulator_set<double, stats<tag::median > > acc;

	accumulator_set<double, stats<tag::median > > accMedianX;
	accumulator_set<double, stats<tag::median > > accMedianY;

	bool calibrationDone;

public:
	Calibrator();

	virtual void initialize();
	virtual void destroy();

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;

	// Threaded SetValue
	void setValueThreadFunc();

};

}

#endif /* CALIBRATOR_HPP_ */
