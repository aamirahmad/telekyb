/*
 * PositionError.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#include <trajmodules/PMPositionError.hpp>

#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_traj::PMPositionError, TELEKYB_NAMESPACE::TrajectoryModule);

namespace telekyb_traj {

PMPositionErrorOptions::PMPositionErrorOptions()
	: OptionContainer("PMPositionError")
{
	tMaxPositionError = addOption("tMaxPositionError", "Maximal Position Error", 0.5, false, true);
}

PMPositionError::PMPositionError()
	: TrajectoryModule("tk_trajprocessor/PMPositionError", TrajModulePosType::Position, 110) // after obs avoid
{

}

void PMPositionError::initialize()
{

}

void PMPositionError::destroy()
{

}

// set back to intial conditions
void PMPositionError::willTurnActive()
{
	//ROS_ERROR("PMPositionError did turn active!");
}

// called after turning inactive
void PMPositionError::didTurnInactive()
{
	//ROS_ERROR("PMPositionError did turn inactive!");
}

bool PMPositionError::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
{
	//ROS_INFO_STREAM("TKState cur: " << std::endl << currentState.position);
	//ROS_INFO_STREAM("TKState tra: " << std::endl << trajInput.position);
	//ROS_INFO("Called PMPositionError");
	// check error
	//ROS_ERROR("PMPositionError trajectoryStep!");
	double posError = (currentState.position - trajInput.position).norm();
	if (posError > options.tMaxPositionError->getValue()) {
		// switch to normalBrake
		ROS_ERROR_STREAM("Trajectory Module " << getName() << ": Position error too big. "
				<< posError << ">" << options.tMaxPositionError->getValue());
		tpController.getBehaviorSwitcher()->switchToNormalBrake();
		trajInput.setVelocity( Velocity3D::Zero() );
		return false; // skip other
	}


	return true;
}

} /* namespace telekyb_traj */
