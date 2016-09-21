/*
 * Experiment.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: pstegagno
 */

#include "Experiment.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>


// Options
ExperimentOptions::ExperimentOptions()
	: OptionContainer("ExperimentOptions")
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
	
}


Experiment::Experiment()
	: mainNodeHandle( ROSModule::Instance().getMainNodeHandle() ), core(NULL)
{
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
// 	hover = bController->getSystemBehavior("tk_behavior/Hover");
	normalBreak = bController->getSystemBehavior("tk_behavior/NormalBrake");
	takeOff = bController->getSystemBehavior("tk_behavior/TakeOff");
	land = bController->getSystemBehavior("tk_behavior/Land");
	

	hover = bController->loadBehavior("tk_be_common/FixedPointHover");
	hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(0.0,0.0,-1.0));
	
	
	normalBreak.getOptionContainer().getOption("tBrakeInPosition").set(true);
	
	normalBreak.setNextBehavior(hover);
      
	
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
	



// 	flyto1 = bController->loadBehavior("tk_be_common/SmoothLinearFlyTo");
// 	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestination").set(Position3D(0.0,0.0,-1.0));
// 	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestinationRadius").set(0.1);
// 	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(0.5);
// 	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(0.5);
	flyto1 = bController->loadBehavior("tk_be_common/PowerFourLinearTrajectoryFlyTo");
	flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToDestination").set(Position3D(0.0,0.0,-1.0));
	flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToDestinationRadius").set(0.1);//was 0.1
	flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToVelocity").set(0.5);
	flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToAcceleration").set(0.5);
// 	flyto1.setNextBehavior(hover);
	flyto1.setParameterInitialized(true);
	

	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// lastly start Controller
	tkstatesub = mainNodeHandle.subscribe(std::string("/clock")
			, 10, &Experiment::clockCB, this);
	
	waypointsub = mainNodeHandle.subscribe(std::string("/firefly/waypoint")
			, 10, &Experiment::waypointCB, this);
	
	startingpositionreached = false;
	interrupted = false;
}



void Experiment::clockCB(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  if (msg->clock.sec > 13) {
    if (*activeBehaviorPtr == ground) {
      bController->switchBehavior(takeOff);
    }
  }
  
  if (msg->clock.sec == 17) {
    if (!startingpositionreached) {
      startingpositionreached = true;
      bController->switchBehavior(flyto1);
    }
  }
}


void Experiment::waypointCB(const mav_msgs::CommandTrajectory::ConstPtr& msg)
{

    // TEST STANDARD SETUP 
//   flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(6.0);  //good values: 6.0, 2.0
//   flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(2.0);
//   flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestination").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));

//   hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
//   bController->switchBehavior(flyto1);



//   TO TEST tPowerFourLinearTrajectoryFlyToVelocity
//   flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToVelocity").set(60.0);  //good values: 6.0, 2.0
//   flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToAcceleration").set(3.2);
//   flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToDestination").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
// 
//   hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
//   bController->switchBehavior(flyto1);



  // to TEST Yuyi controller
  flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToVelocity").set(2.0);  //good values: 6.0, 2.0
  flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToAcceleration").set(1.0);
  flyto1.getOptionContainer().getOption("tPowerFourLinearTrajectoryFlyToDestination").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));

  hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
  bController->switchBehavior(flyto1);
  
  
//   hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
}


void Experiment::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
}

