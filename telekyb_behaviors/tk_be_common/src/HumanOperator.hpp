#ifndef HUMANOPERATOR_HPP_
#define HUMANOPERATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>
#include <telekyb_msgs/TKCommands.h>
#include <std_msgs/Bool.h>
#include <telekyb_interface/MKInterface.hpp>
#include <telekyb_interface/TeleKybCore.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {
  
  
// class HumanOperatorOptions{
//   
// };

class HumanOperator : public OptionContainer{
public:
	Option<std::string>* tHumanOperatorTopic;
	Option<std::string>* tTelekybCommandTopic;
	Option<double>* tHumanOperatorYawRateScale;

	//input is interpreted in local frame!
	//disable dead man switch?
	Option<bool>* tHumanOperatorUseDeadManSwitch;
	
	Option<int>* tRobotID;
	
	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber joySub;
	ros::Subscriber tkSub;
	
	// Optional MKInterface
	telekyb_interface::MKInterface* mkInterface;

	void tkCommandsCB(const telekyb_msgs::TKCommands::ConstPtr& msg);
	void humanCB(const sensor_msgs::Joy::ConstPtr& msg);

	// Integrated Position for Velocity Mode
	Angle posModeCurYawAngle;
	Time posModeLastInputTime;

	// Outputfield
	bool active, yawTrimmerInitialized;
	double receivedMass, receivedThrust, hoveringThrust, yawTrimmer;
	MKSingleValuePacket *motorState;

	
	


	HumanOperator();

	void initialize();
	virtual void destroy();

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior){ return true; }
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior){}
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior){}
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior){}

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput){}

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput){}

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput){}

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const{ return true; }
};

}
#endif /* JOYSTICK_HPP_ */
