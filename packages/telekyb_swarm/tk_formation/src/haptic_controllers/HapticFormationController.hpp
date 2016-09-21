/*
 * HapticFormationController.hpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#ifndef HAPTICFORMATIONCONTROLLER_HPP_
#define HAPTICFORMATIONCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_haptics_base/HapticDeviceController.hpp>

#include <telekyb_base/Time.hpp>

#include <telekyb_base/Options.hpp>

// Send Vector3
#include <geometry_msgs/Vector3Stamped.h>

#include <telekyb_msgs/TKState.h>
#include <telekyb_base/Messages.hpp>
// QC Feedback

// Generic Subscribers
#include <telekyb_base/ROS/GenericSubscriber.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_haptic {

class HapticFormationControllerOptions : public OptionContainer
{
public:
	Option< std::vector<int> >* tRobotIDs;
	Option< double >* tSendFrequency;
	Option< std::string >* tInputTopicName;
	Option< std::string >* tOutputTopicName;
	Option< double >* tVelocityGain;
	Option< Eigen::Matrix3d >* tRotationMatrix;
	// Gains
	Option< double >* tPropGain;
	Option< double >* tDeriGain;
	Option< double >* tErrGain;


	HapticFormationControllerOptions(const std::string& identifier);
};



class HapticFormationController : public HapticDeviceController {
protected:
	Timer frequencyTimer;
	HapticFormationControllerOptions* options;

	// Pub and Sub
	ros::NodeHandle nodeHandle;
	ros::Publisher vectorPub;

	std::vector< GenericSubscriber<telekyb_msgs::TKState>* > stateSubscribers;

	ros::Subscriber stateSub0;
	ros::Subscriber stateSub1;

	// Last Msgs
	TKState lastStateMsg0;
	TKState lastStateMsg1;

	void stateCB0(const telekyb_msgs::TKState::ConstPtr& stateMsg);
	void stateCB1(const telekyb_msgs::TKState::ConstPtr& stateMsg);


	//Eigen::Matrix3d rotation;



public:
	HapticFormationController();
	virtual ~HapticFormationController();

	// Identifier (e.g. for NodeHandle)
	void setIdentifier(const std::string& identifier);

	// Get's specific Axes mapping, Set if needed
	void setAxesMapping(HapticAxesMapping& xAxis, HapticAxesMapping& yAxis, HapticAxesMapping& zAxis);

	// Get the Range of each axes
	void setAxesRange(const Position3D& minValues, const Position3D& maxValues);

	void willEnterSpinLoop();

	// has to be fast and should not slow down the loop
	void loopCB(const HapticOuput& output, HapticInput& input);

	void didLeaveSpinLoop();
};

}

#endif /* HAPTICFORMATIONCONTROLLER_HPP_ */
