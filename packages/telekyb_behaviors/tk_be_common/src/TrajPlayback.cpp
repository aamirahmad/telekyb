/*
 * TrajPlayback.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "TrajPlayback.hpp"

// boost
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>


// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::TrajPlayback, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

TrajPlayback::TrajPlayback()
	: Behavior("tk_be_common/TrajPlayback", BehaviorType::Air)
{

}

void TrajPlayback::initialize()
{
	tTrajectoryFilename = addOption<std::string>("tTrajectoryFilename",
			"File with Trajectory Information",
			"Traj.txt",
			false,false);

    tPositionThreshold = addOption<double>("tPositionThreshold",
            "Maximum Position Error to switch to the TrajPlayback",
            0.5, false, false);
	//parameterInitialized = true;
}

void TrajPlayback::destroy()
{

}

bool TrajPlayback::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// TODO: Cannot activate TrajPlayback Behavior while Flying.

	ROS_INFO_STREAM("Playing back file with name " << tTrajectoryFilename->getValue());

	file.open(tTrajectoryFilename->getValue().c_str());

	if (!file.is_open()) {
		ROS_ERROR_STREAM("Unable to open Trajectory File: " << tTrajectoryFilename->getValue());
		return false;
	}

	// ok it's open
	if (!setNextTrajInput()) {
		file.close();
		return false;
	}

    double positionThreshold = tPositionThreshold->getValue();
//    double positionThreshold = 0.5;
    double velocityThreshold = 0.01;
	double positionError = (currentState.position - nextTrajInput.position).norm();

	double velocityError = (currentState.linVelocity - nextTrajInput.velocity).norm();
	if (positionError > positionThreshold){// || velocityError  > velocityThreshold) {
		ROS_ERROR_STREAM("Current Position: " << currentState.position.transpose() << " First Traj: " << nextTrajInput.position.transpose());
		ROS_ERROR("Conditions not met: position error %f; velocity error: %f", positionError, velocityError);
		file.close();
		return false;
	}

	return true;
}

void TrajPlayback::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	timer.reset();

}

void TrajPlayback::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void TrajPlayback::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// hover does not need this
	ROS_INFO("TrajPlayback became inactive");
	file.close();
}

bool TrajPlayback::setNextTrajInput()
{
	std::string line;
	getline(file,line);

	std::vector<std::string> tokenList;
	boost::split(tokenList, line, boost::is_any_of(" \t"), boost::token_compress_on);

/*	for (int i = 0; i < tokenList.size(); ++i) {
		double temp = 0;
		try {
			temp = boost::lexical_cast<double>(tokenList[i]);
		} catch (boost::bad_lexical_cast& e) {
			ROS_ERROR("Casting error: %s", e.what());
		}

		ROS_INFO("Token Number: %d Value: %f", i , temp);
	}*/

	if (tokenList.size() == 12) {
		//ROS_INFO("Found 12 tokens per line, will use old trajectory file definition without jerk and snap.");
		try {
			nextTimeStep = boost::lexical_cast<double>(tokenList[0]);
	
			Position3D position(
					boost::lexical_cast<double>(tokenList[1]),
					boost::lexical_cast<double>(tokenList[2]),
					boost::lexical_cast<double>(tokenList[3])
			);

			Velocity3D velocity(
					boost::lexical_cast<double>(tokenList[4]),
					boost::lexical_cast<double>(tokenList[5]),
					boost::lexical_cast<double>(tokenList[6])
			);
	
			Acceleration3D acceleration(
					boost::lexical_cast<double>(tokenList[7]),
					boost::lexical_cast<double>(tokenList[8]),
					boost::lexical_cast<double>(tokenList[9])
			);

			Eigen::Vector3d jerk(0,0,0);

			Eigen::Vector3d snap(0,0,0);

			nextTrajInput.setPosition(position, velocity, acceleration, jerk, snap);
			nextTrajInput.setYawAngle(
					boost::lexical_cast<double>(tokenList[10]),
					boost::lexical_cast<double>(tokenList[11]),
					0
					);

		} catch (boost::bad_lexical_cast& e) {
			ROS_ERROR("Casting error: %s", e.what());
			return false;
		}
		return true;
	}


	if (tokenList.size() < 19) {
		ROS_ERROR("Line has less than 19 Tokens!");
		return false;
	}
	try {
		//ROS_INFO("Found 19 or more tokens per line, will use new trajectory file definition including jerk and snap.");
		nextTimeStep = boost::lexical_cast<double>(tokenList[0]);

		Position3D position(
				boost::lexical_cast<double>(tokenList[1]),
				boost::lexical_cast<double>(tokenList[2]),
				boost::lexical_cast<double>(tokenList[3])
		);

		Velocity3D velocity(
				boost::lexical_cast<double>(tokenList[4]),
				boost::lexical_cast<double>(tokenList[5]),
				boost::lexical_cast<double>(tokenList[6])
		);

		Acceleration3D acceleration(
				boost::lexical_cast<double>(tokenList[7]),
				boost::lexical_cast<double>(tokenList[8]),
				boost::lexical_cast<double>(tokenList[9])
		);

		Eigen::Vector3d jerk(
				boost::lexical_cast<double>(tokenList[10]),
				boost::lexical_cast<double>(tokenList[11]),
				boost::lexical_cast<double>(tokenList[12])
		);

		Eigen::Vector3d snap(
				boost::lexical_cast<double>(tokenList[13]),
				boost::lexical_cast<double>(tokenList[14]),
				boost::lexical_cast<double>(tokenList[15])
		);

		nextTrajInput.setPosition(position, velocity, acceleration, jerk, snap);
		nextTrajInput.setYawAngle(
				boost::lexical_cast<double>(tokenList[16]),
				boost::lexical_cast<double>(tokenList[17]),
				boost::lexical_cast<double>(tokenList[18])
				);

	} catch (boost::bad_lexical_cast& e) {
		ROS_ERROR("Casting error: %s", e.what());
		return false;
	}

	return true;
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void TrajPlayback::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void TrajPlayback::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	//ROS_INFO("TrajStep Called!");
    if (timer.getElapsed().toDSec() > nextTimeStep) {
		setNextTrajInput();
	}


	generatedTrajInput = nextTrajInput;
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void TrajPlayback::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool TrajPlayback::isValid(const TKState& currentState) const
{
	// never turns invalid
	return file.good();
}

}

