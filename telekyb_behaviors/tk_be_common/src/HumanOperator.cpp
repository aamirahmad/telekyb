#include <telekyb_base/ROS.hpp>
#include "HumanOperator.hpp"
// Declare
// PLUGINLIB_EXPORT_CLASS( telekyb_behavior::HumanOperator, TELEKYB_NAMESPACE::Behavior);

telekyb_msgs::TKCommands cmd;
std_msgs::Bool interruptValue;

namespace telekyb_behavior {  

HumanOperator::HumanOperator(): OptionContainer("HumanOperator"),
		mkInterface(NULL),
		tHumanOperatorTopic(NULL),
		tHumanOperatorUseDeadManSwitch(NULL),
		tHumanOperatorYawRateScale(NULL),
		tTelekybCommandTopic(NULL),
		tRobotID(NULL)
{
	motorState = (MKSingleValuePacket*) new MKSingleValuePacket(MKDataDefines::MOTOR_STATE,0);
	active = false;
	yawTrimmer = 0;
	yawTrimmerInitialized = false;
	receivedThrust = 0;
	hoveringThrust = 0;
	receivedMass = 0;
}

void HumanOperator::tkCommandsCB(const telekyb_msgs::TKCommands::ConstPtr& msg)
{
	receivedMass = msg->mass;
	receivedThrust = msg->thrust;
}

void HumanOperator::humanCB(const sensor_msgs::Joy::ConstPtr& msg)
{
//	ROS_INFO("Received HumanOperator CB");

	// invalidate
	if ( msg->buttons[1] ) {
		
		if (active){
			active = false;
			interruptValue.data = false;
		}
		else {
			active = true;
			interruptValue.data = true;
			if (!mkInterface->updateMKValue(*motorState)) {
				ROS_ERROR("DANGER!!! Could not get Motorstate while trying to activate Human operator!");
				return;
			}
			hoveringThrust = receivedThrust;
// 			Initialize the yaw trimmer only once
			if (!yawTrimmerInitialized) {
				yawTrimmer = 0;
				yawTrimmerInitialized = true;
			}
			cmd.mass = receivedMass;
			
		}
	}


	if (active) {
		if (mkInterface) {
			if (msg->buttons[2]) {
				MKSingleValuePacket flightControlMode(MKDataDefines::FLIGHT_CTRL_MODE,0);
				if (!mkInterface->updateMKValue(*motorState)) {
					ROS_ERROR("Could not get Motorstate for liftoff!");
					return;
				}
				if (motorState->value == MotorState::On) {
					// stop
					motorState->value = MotorState::Off;
					mkInterface->setMKValue(*motorState);
				} else if (motorState->value == MotorState::Off) {
					// start
					motorState->value = MotorState::Init;
					mkInterface->setMKValue(*motorState);
					hoveringThrust = -5.0;
					yawTrimmer = 0;
				} else if (motorState->value == MotorState::Init) {
					motorState->value = MotorState::On;
					flightControlMode.value = 0;
					mkInterface->setMKValue(flightControlMode);
					mkInterface->setMKValue(*motorState);
					
				}
	// 			if (motorState.value == MotorState::On) {
	// 				bController->switchBehavior(takeOff);
	// 			} else {
	// 				ROS_ERROR("Motors have to be on for liftOff!");
	// 				return;
	// 			}
			}
			
			if (msg->axes.size() < 4) {
				ROS_ERROR("We need at least 4 Axes!");
				return;
			}

			if (msg->buttons.size() < 2) {
				ROS_ERROR("We need at least 2 Buttons!");
				return;
			}

			if (motorState->value == MotorState::On){
				
// 				Roll commands
				if (msg->axes[1] < 0.18 && msg->axes[1] > -0.18) {
					cmd.roll = -msg->axes[1];
				} else if (msg->axes[1] > 0) {
					cmd.roll = -0.18; 
				} else {
					cmd.roll = 0.18;
				}
				
// 				Pitch commands
				if (msg->axes[0] < 0.18 && msg->axes[0] > -0.18) {
					cmd.pitch = msg->axes[0];
				} else if (msg->axes[0] > 0) {
					cmd.pitch = 0.18; 
				} else {
					cmd.pitch = -0.18;
				}
				
// 				Thrust commands (manage the hovering thrust)
				if (msg->axes[7] > 0 && hoveringThrust - 0.1 > -10.0) {
					hoveringThrust -= 0.1; 
					cmd.thrust = hoveringThrust;
				} else if (msg->axes[7] < 0 && hoveringThrust + 0.1 < -5.5) {
					hoveringThrust += 0.1; 
					cmd.thrust = hoveringThrust;
				}
				
// 				Thrust commands (command the thrust with the analogic lever)
				if (msg->axes[2] >= 0) { 
					if (msg->axes[2] < 0.7) {
						cmd.thrust = hoveringThrust + msg->axes[2];
					} else {
						cmd.thrust = hoveringThrust + 0.7;
					}
				} else if (msg->axes[2] > -0.7) {
					cmd.thrust = hoveringThrust - msg->axes[2];
				} else {
					cmd.thrust = hoveringThrust - 0.7;
				}
				
// 				yawTrimmer managment
				if (msg->axes[6] > 0 && yawTrimmer - 0.025 > -0.7) {
					yawTrimmer -= 0.025; 
					cmd.yaw = yawTrimmer;
				} else if (msg->axes[6] < 0 && yawTrimmer + 0.025 < 0.7) {
					yawTrimmer += 0.025; 
					cmd.yaw = yawTrimmer;
				}
				
// 				Yaw commands
				if (msg->axes.size() != 4) {
					cmd.yaw = yawTrimmer - (msg->axes[4] - msg->axes[3])/2.0 * tHumanOperatorYawRateScale->getValue();
				} else {
					// 4 axes case
					cmd.yaw = yawTrimmer -  msg->axes[3] * tHumanOperatorYawRateScale->getValue();
				}
				

				cmd.mass = receivedMass;
			}
		}
	}
}

void HumanOperator::initialize()
{
	tHumanOperatorTopic = addOption<std::string>("tHumanOperatorTopic",
			"HumanOperatorTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, false);
	
	tTelekybCommandTopic = addOption<std::string>("tTelekybCommandTopic",
			"tTelekybCommandTopic that published telekyb_msgs::TKCommands",
			"/TeleKyb/Commands", false, false);

// 	tHumanOperatorUsePositionMode = addOption("tHumanOperatorUsePositionMode",
// 			"Integrates Position from Velocity input.", true, false, false);
	tHumanOperatorYawRateScale = addOption("tHumanOperatorYawRateScale",
			"Commanded Yaw Rate is scaled to this value", 1.0, false ,false);

// 	tHumanOperatorUseRelativeMode = addOption("tHumanOperatorUseRelativeMode",
// 			"Enable this to have all inputs interpreted w.r.t. the local frame.", false, false, true);
	tHumanOperatorUseDeadManSwitch = addOption("tHumanOperatorUseDeadManSwitch",
			"Disable this to fly without pressing a special button simultaneously.", true, false, true);
	
	tRobotID = addOption<int>("tRobotID", "ID of your robot", 7, false, false);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	
// 	joySub = nodeHandle.subscribe(tHumanOperatorTopic, 1, humanCB, this);
	
	joySub = nodeHandle.subscribe(tHumanOperatorTopic->getValue()
			, 10, &HumanOperator::humanCB, this);
	
	tkSub = nodeHandle.subscribe(tTelekybCommandTopic->getValue()
			, 10, &HumanOperator::tkCommandsCB, this);

	mkInterface = telekyb_interface::MKInterface::getMKInterface(tRobotID->getValue());
		//mkInterface = telekyb_interface::MKInterface::getMKInterface(1); // BEWARE TEMPORARY!!!
		if (!mkInterface) {
			// fail
			ros::shutdown();
			return;
		}
	yawTrimmerInitialized = false;
	// no Parameters
	//parameterInitialized = true;
}

void HumanOperator::destroy()
{
}


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "humanOperator");
	
	telekyb_behavior::HumanOperator *human = new telekyb_behavior::HumanOperator();
	human->initialize();
// 	joySub = human->nodeHandle.subscribe(human->tHumanOperatorTopic, 1, &HumanOperator::humanCB);

	ros::Publisher commandsPublisher = human->nodeHandle.advertise<telekyb_msgs::TKCommands>("/TeleKyb/mkinterface_outdoor_7/RC_Commands", 1);
	ros::Publisher interruptPublisher = human->nodeHandle.advertise<std_msgs::Bool>("/TeleKyb/mkinterface_outdoor_7/interrupt", 1);
	interruptValue.data = false;

	ros::Rate loop_rate(20);
	while (ros::ok()) {
		interruptPublisher.publish(interruptValue);
		commandsPublisher.publish(cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

