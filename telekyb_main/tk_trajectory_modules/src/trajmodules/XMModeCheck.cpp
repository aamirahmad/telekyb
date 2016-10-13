/*
 * XMModeCheck.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <trajmodules/XMModeCheck.hpp>

#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_traj::XMModeCheck, TELEKYB_NAMESPACE::TrajectoryModule);


namespace telekyb_traj {

XMModeCheckOptions::XMModeCheckOptions()
	: OptionContainer("XMModeCheck")
{
	tAllowPositionMode = addOption("tAllowPositionMode", "Allow or disallow Position Mode",true,false,false);
	tAllowVelocityMode = addOption("tAllowVelocityMode", "Allow or disallow Velocity Mode",true,false,false);
	tAllowAccelerationMode = addOption("tAllowAccelerationMode", "Allow or disallow Acceleration Mode",false,false,false);
	tAllowMixedModes = addOption("tAllowMixedModes", "Allow or disallow Mixed Modes",false,false,false);
}

XMModeCheck::XMModeCheck()
	: TrajectoryModule("tk_trajprocessor/XMModeCheck", TrajModulePosType::All, -500)
{

}

void XMModeCheck::initialize()
{

}

void XMModeCheck::destroy()
{

}

// set back to intial conditions
void XMModeCheck::willTurnActive()
{

}

// called after turning inactive
void XMModeCheck::didTurnInactive()
{

}

bool XMModeCheck::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
{
	//ROS_INFO("Called XMModeCheck");
	bool doNormalBrake = false;
	GlobalPosControlType inputPosCtrlType = trajInput.getGlobalPositionControlType();
	if (inputPosCtrlType == GlobalPosControlType::Position && !options.tAllowPositionMode->getValue()) {
		ROS_ERROR_STREAM("Trajectory Module " << getName() << ": Disallowed PositionControlMode: Position!");
		doNormalBrake = true;
	} else if (inputPosCtrlType == GlobalPosControlType::Velocity && !options.tAllowVelocityMode->getValue()) {
		ROS_ERROR_STREAM("Trajectory Module " << getName() << ": Disallowed PositionControlMode: Velocity!");
		doNormalBrake = true;
	} else if (inputPosCtrlType == GlobalPosControlType::Acceleration && !options.tAllowAccelerationMode->getValue()) {
		ROS_ERROR_STREAM("Trajectory Module " << getName() << ": Disallowed PositionControlMode: Acceleration!");
		doNormalBrake = true;
	} else if (inputPosCtrlType == GlobalPosControlType::Mixed && !options.tAllowMixedModes->getValue()) {
		ROS_ERROR_STREAM("Trajectory Module " << getName() << ": Disallowed PositionControlMode: Mixed!");
		doNormalBrake = true;
	}

	if (doNormalBrake) {
		tpController.getBehaviorSwitcher()->switchToNormalBrake();
		trajInput.setVelocity( Velocity3D::Zero() );
		return false; // skip other
	}


	return true;
}

} /* namespace telekyb_traj */
