/*
 * EurocTrajectoryTracker.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include "EurocTrajectoryTracker.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKCommands.h>
#include <rosgraph_msgs/Clock.h>

//
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_defines/physic_defines.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::EurocTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {


// Options
EurocTrajectoryTrackerOptions::EurocTrajectoryTrackerOptions()
	: OptionContainer("EurocTrajectoryTracker")
{
	tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for publishing telekyb_msgs::TKCommands",
			"commands", false, true);
	tPluginLookupName = addOption<std::string>("tPluginLookupName",
			"Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
			"parameter_estimators_plugin::StandardMassEstimator", false, true);
	tTaskNumber = addOption<int>("tTaskNumber",
			"Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
			31, false, true);
	tExternalForceTopic = addOption<std::string>("tExternalForceTopic",
			"specifies the topic with the external force",
			"/Telekyb/ForceDisturbance", false, true);
	tCommandedThurstTopic = addOption<std::string>("tCommandedThurstTopic",
			"specifies the topic with the commanded thrust",
			"/Telekyb/ControlInputU1", false, true);

}


EurocTrajectoryTracker::EurocTrajectoryTracker()
	: tDoMassEstimation( NULL ),
	  meLoader( "tk_param_estimator", "tk_param_estimator::MassEstimator" ),
// 	  massEstimator( NULL ),
	  nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
	  commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX )
{
// 	clocksub = nodeHandle.subscribe(std::string("/clock")
// 			, 10, &EurocTrajectoryTracker::clockcb, this);
	extforcesub = nodeHandle.subscribe(options.tExternalForceTopic->getValue()
			, 10, &EurocTrajectoryTracker::extforcecb, this);
	commandedthrustsub = nodeHandle.subscribe(options.tCommandedThurstTopic->getValue()
			, 10, &EurocTrajectoryTracker::thrustcb, this);
	additional_pitch = 0.0;
	additional_roll = 0.0;
	commanded_thrust = 0.0;
}


// // void Experiment::tkstateCB(const telekyb_msgs::TKState::ConstPtr& msg)
// void EurocTrajectoryTracker::clockcb(const rosgraph_msgs::Clock::ConstPtr& msg)
// {
//   switch (options.tTaskNumber->getValue()){
//     case 31:
//       additional_pitch = 0.0;
//       additional_roll = 0.0;
//       break;
//     case 32:
//       if (msg->clock.sec >= 40) {
// 	additional_pitch = 0.08;
// 	additional_roll = 0.08;
//       }
//       break;
//     case 33:
//       break;
//     case 41:
//       break;
//     case 42:
//       break;
//     case 43:
//       break;
//     default:
//       break;
//   }
// }




void EurocTrajectoryTracker::extforcecb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  switch (options.tTaskNumber->getValue()){
    case 31:
      additional_pitch = 0.0;
      additional_roll = 0.0;
      break;
    case 32:
      if ( msg->header.stamp.toSec() >= 30.0 ) {
	if (abs(commanded_thrust) > 3.0) {
  // 	std::cout << "commanded_thrust  " << commanded_thrust <<  "     msg->vector.x  " << msg->vector.x << std::endl;
	  additional_pitch = asin(msg->vector.x/commanded_thrust);
	  additional_roll = asin(msg->vector.y/commanded_thrust);
	  std::cout << "  " << additional_pitch <<  "     " << additional_roll << std::endl;
    // 	additional_pitch = 0.08;
    // 	additional_roll = 0.08;
	}
      }

      else {
	additional_pitch = 0.0;
	additional_roll = 0.0;
// 	std::cout << "commanded_thrust == 0.0  " << std::endl;
      }
      break;
    case 33:
      if ( msg->header.stamp.toSec() >= 30.0 ) {
	if (abs(commanded_thrust) > 3.0) {
  // 	std::cout << "commanded_thrust  " << commanded_thrust <<  "     msg->vector.x  " << msg->vector.x << std::endl;
	  additional_pitch = asin(msg->vector.x/commanded_thrust);
	  additional_roll = asin(msg->vector.y/commanded_thrust);
	  std::cout << "  " << additional_pitch <<  "     " << additional_roll << std::endl;
    // 	additional_pitch = 0.08;
    // 	additional_roll = 0.08;
	}
      }

      else {
	additional_pitch = 0.0;
	additional_roll = 0.0;
// 	std::cout << "commanded_thrust == 0.0  " << std::endl;
      }
      break;
    case 41:
      break;
    case 42:
      break;
    case 43:
      break;
    default:
      break;
  }
}


void EurocTrajectoryTracker::thrustcb(const std_msgs::Float64::ConstPtr& msg)
{
	commanded_thrust = msg->data;
}


EurocTrajectoryTracker::~EurocTrajectoryTracker()
{
	if (massEstimator) {
		massEstimator->destroy();
// 		delete massEstimator;
	}
}

void EurocTrajectoryTracker::initialize()
{
	// Important first create Publisher, before receiving CallBacks
	tTcCommandsPub = commandNodeHandle.advertise<telekyb_msgs::TKCommands>(options.tCommandsTopic->getValue(),1);

	// CurrentState
	currentInput.setAcceleration( TELEKYB_NAMESPACE::Acceleration3D(0.0, 0.0, GRAVITY) );
	currentInput.setYawRate(0.0);

	//std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
	//tTcStateSub = nodeHandle.subscribe(tkStateTopicName,1,&EurocTrajectoryTracker::tkStateCB, this);
	positionControl = new TELEKYB_NAMESPACE::PositionControl;
	yawControl = new TELEKYB_NAMESPACE::YawControl();

	try {
		massEstimator = meLoader.createInstance(options.tPluginLookupName->getValue());
		// Currently RunTime Switch is not supported. This has to be changed then.
		massEstimator->initialize();

	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL("Trajectory Tracker %s failed to load: %s", options.tPluginLookupName->getValue().c_str(), e.what());
		//ROS_BREAK();
		ros::shutdown();
	}

	// Get Option
	tDoMassEstimation = TELEKYB_NAMESPACE::OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoMassEstimation");
	if (!tDoMassEstimation) {
		ROS_ERROR("Unable to get Option TrajectoryController/tDoMassEstimation. Quitting...");
		ros::shutdown();
	}

	// fill currentMass with Initial Value!
	currentMass = massEstimator->getInitialMass();

	// sanity check
//	TrajectoryTracker *tracker = new EurocTrajectoryTracker();
//	delete tracker;
}
//	virtual void willBecomeActive() = 0;
//	virtual void willBecomeInActive() = 0;
void EurocTrajectoryTracker::destroy()
{

}

std::string EurocTrajectoryTracker::getName() const
{
	return "EurocTrajectoryTracker";
}


void EurocTrajectoryTracker::trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory)
{
	boost::mutex::scoped_lock currentInputLock(currentInputMutex);
	currentInput = trajectory;
}

void EurocTrajectoryTracker::stateCB(const TELEKYB_NAMESPACE::TKState& state)
{
	// new State Message. Triggers control step!
	//ROS_INFO("Received new TKState!");
	TELEKYB_NAMESPACE::Vector3D rpyOrientation = state.getEulerRPY();

	// Position Control
	TELEKYB_NAMESPACE::PosCtrlOutput pcOutput;
	// lock
	boost::mutex::scoped_lock currentInputLock(currentInputMutex);
  
	positionControl->run(currentInput, state, currentMass, pcOutput);
	// Yaw Control
	TELEKYB_NAMESPACE::YawCtrlOutput ycOutput;
	yawControl->run(currentInput, state, ycOutput);
	
	// unlock
	currentInputLock.unlock();



	//ROS_INFO("Roll: %f, Pitch: %f, Thrust %f", pcOutput.comRoll, pcOutput.comPitch, pcOutput.comThrust);

	// Mass Estimation only when enabled.
	if (tDoMassEstimation->getValue()) {
		MassEstimInput meInput;
		meInput.roll = rpyOrientation(0);
		meInput.pitch = rpyOrientation(1);
		meInput.thrust = pcOutput.comThrust;
		meInput.vertVel = state.linVelocity(2); // z

		MassEstimOutput meOutput;
		massEstimator->run(meInput, meOutput);
		// update Mass.
		currentMass = meOutput.estMass;
	}


	//positionControl.setMass(meOutput.estMass);


	telekyb_msgs::TKCommands cmdMsg;

	cmdMsg.header.stamp = ros::Time::now();
// 	std::cout << "  " << additional_pitch <<  "     " << additional_roll << std::endl;
	cmdMsg.roll = pcOutput.comRoll+additional_roll;
	cmdMsg.pitch = pcOutput.comPitch+additional_pitch;
	cmdMsg.yaw = ycOutput.comYaw;
	cmdMsg.thrust = pcOutput.comThrust;
	// mass
	cmdMsg.mass = currentMass;


	tTcCommandsPub.publish(cmdMsg);

}


}
