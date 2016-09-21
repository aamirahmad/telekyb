/*
 * Experiment.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: pstegagno
 */

#include "Experiment.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>
#include <telekyb_base/Messages.hpp>

#include <telekyb_base/Spaces.hpp>

// Options
ExperimentOptions::ExperimentOptions(): OptionContainer("ExperimentOptions")
{
	robotID = addOption<int>("robotID", "Specify the robotID of the TeleKybCore to connect to.", 0, false, true);
	tOmega6JoyTopic = addOption<std::string>("tOmega6JoyTopic",
			"Omega6JoyTopictopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tInterruptTopic = addOption<std::string>("tInterruptTopic",
			"interrupt topic to use (std_msgs::Bool)", "/mkinterface_outdoor/humanOperator/interrupt", false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tCommandedYawRateTopic = addOption<std::string>("tCommandedYawRateTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tYawSinComponentTopic = addOption<std::string>("tYawSinComponentTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tSinPulse = addOption("tSinPulse",
			"Pulse for the yaw motion", 1.0, false ,false);
	tSinAmplitude = addOption("tSinAmplitude",
			"Amplitude for the yaw motion", 0.5, false ,false);
    //tTKStateTopic = addOption<std::string>("tTKStateTopic", "Name of the TKState topic","/TeleKyb/TeleKybCore_0/Sensor/TKState", true, true);
	
}


Experiment::Experiment()
	: mainNodeHandle( ROSModule::Instance().getMainNodeHandle() ), core(NULL)
{
    /*
     *  set logger level
    */
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

	core = telekyb_interface::TeleKybCore::getTeleKybCore(options.robotID->getValue());
	if (!core) {
		// fail
		ros::shutdown();
		return;
	}

	bController = core->getBehaviorController();
	oController = core->getOptionController();


	//activeBehavior = bController->getActiveBehaviorReference();
	bController->setActiveBehaviorListener(this);

	activeBehaviorPtr = bController->getActiveBehaviorPointer();

	setupExperiment();
}

Experiment::~Experiment()
{
	delete core;
}

void Experiment::setupExperiment()
{
	// load Behaviors
	ground = bController->getSystemBehavior("tk_behavior/Ground");
	hover = bController->getSystemBehavior("tk_behavior/Hover");
	normalBreak = bController->getSystemBehavior("tk_behavior/NormalBrake");
	takeOff = bController->getSystemBehavior("tk_behavior/TakeOff");
	land = bController->getSystemBehavior("tk_behavior/Land");


	// sanity check
	if (ground.isNull() || hover.isNull() || normalBreak.isNull() || takeOff.isNull() || land.isNull() ) {
		ROS_FATAL("Unable to get SystemBehavior!!!");
		//ROS_BREAK();
		ros::shutdown();
	}

	// done
	takeOff.setParameterInitialized(true);

	land.setParameterInitialized(true);
	land.setNextBehavior(ground);
	
	
	
	fixedPointHover = bController->loadBehavior("tk_be_common/FixedPointHover");
	fixedPointHover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(0.0,0.0,-1.0));
	
	fixedPointHover.setParameterInitialized(true);
	
	
	

	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// lastly start Controller
    clocksub = mainNodeHandle.subscribe("/clock", 1, &Experiment::clockCB, this);
    tkstatesub = mainNodeHandle.subscribe("/TeleKyb/TeleKybCore_0/Sensor/TKState", 1, &Experiment::tkstateCB, this);

	

	interrupted = false;
}




void Experiment::tkstateCB(const telekyb_msgs::TKState::ConstPtr& msg){
    TKState state;
    state.fromTKStateMsg(*msg);
    telekyb::Vector3D euler = state.getEulerRPY();

    ROS_DEBUG_STREAM_THROTTLE(1,mainNodeHandle.getNamespace() << " euler " << euler[0] << " " <<  euler[1] << " " <<  euler[2]);

}

void Experiment::clockCB(const rosgraph_msgs::Clock::ConstPtr& msg)
{
//   ROS_ERROR("a %d", msg->clock.sec);

  if (msg->clock.sec > 13) {
    if (*activeBehaviorPtr == ground) {
      bController->switchBehavior(takeOff);
    }
  }
  
  if (msg->clock.sec == 17) {
    if (*activeBehaviorPtr != fixedPointHover) {
      bController->switchBehavior(fixedPointHover);
    }
  }
}



void Experiment::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
}

