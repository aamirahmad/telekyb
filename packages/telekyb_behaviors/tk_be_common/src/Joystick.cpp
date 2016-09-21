/*
 * Joystick.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "Joystick.hpp"

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::Joystick, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {


Joystick::Joystick()
	: Behavior("tk_be_common/Joystick", BehaviorType::Air),
	  tJoystickTopic(NULL),
	  tJoystickUsePositionMode(NULL),
	  tJoystickYawRateScale(NULL),
	  tJoystickUseDeadManSwitch(NULL),
	  tJoystickUseRelativeMode(NULL)
{
	valid = false;
	lastYawRateInput = 0;
}

void Joystick::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
    //	ROS_INFO("Received Joystick CB");

    if (msg->axes.size() < 4 && msg->buttons.size() < 2) {
        ROS_ERROR("We need at least 4 Axes and at least 2 Buttons");
        return;
    }


	//dead man button is pressed, or we do not care about the dead man button
    if ( msg->buttons[tJoystickCommand_DEADMANSWITCH->getValue()] || !tJoystickUseDeadManSwitch->getValue()) {
        lastVelocityInput = Velocity3D(msg->axes[tJoystickCommand_FORBACK->getValue()]*tJoystickXScale->getValue(),
                                       msg->axes[tJoystickCommand_LEFTRIGHT->getValue()]*-tJoystickYawRateScale->getValue(),
                                       msg->axes[tJoystickCommand_UPDOWN->getValue()]*-tJoystickZScale->getValue());

            lastYawRateInput = (msg->axes[tJoystickCommand_YAW->getValue()] * -tJoystickYawRateScale->getValue());
	} else {
		// apply 0
		lastVelocityInput = Velocity3D::Zero();
		lastYawRateInput = 0.0;
	}


	// invalidate
    if ( msg->buttons[tJoystickCommand_DISABLE->getValue()] ) {
		valid = false;
	}
}



void Joystick::initialize()
{
	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"JoystickTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, false);

	tJoystickUsePositionMode = addOption("tJoystickUsePositionMode",
			"Integrates Position from Velocity input.", true, false, false);
	tJoystickYawRateScale = addOption("tJoystickYawRateScale",
			"Commanded Yaw Rate is scaled to this value", 1.0, false ,false);

    tJoystickZScale = addOption<double>("tJoystickZScale","Scaling factor for the thrust or z Axis input",1.0, false, false);
    tJoystickYScale = addOption<double>("tJoystickYScale","Scaling factor for the y Axis input",1.0, false, false);
    tJoystickXScale = addOption<double>("tJoystickXScale","Scaling factor for the x Axis input",1.0, false, false);

    tJoystickCommand_YAW = addOption<int>("tJoystickCommand_YAW",
                                                 "Axes on the joystick to control yaw rotation",
                                                 0,false,false);
    tJoystickCommand_UPDOWN = addOption<int>("tJoystickCommand_UPDOWN",
                                                 "Axes on the joystick to control up and down motion",
                                                 1,false,false);
    tJoystickCommand_LEFTRIGHT = addOption<int>("tJoystickCommand_LEFTRIGHT",
                                                 "Axes on the joystick to control left and right motion",
                                                 3,false,false);
    tJoystickCommand_FORBACK = addOption<int>("tJoystickCommand_FORBACK",
                                                 "Axes on the joystick to control forward and backward motion",
                                                 4,false,false);

    tJoystickCommand_DISABLE = addOption<int>("tJoystickCommand_DISABLE","Button on the joystick to disable",
                                                 5,false,false);
    tJoystickCommand_DEADMANSWITCH = addOption<int>("tJoystickCommand_DEADMANSWITCH","the deadmanswtich button on the joystick",
                                                 6,false,false);



	tJoystickUseRelativeMode = addOption("tJoystickUseRelativeMode",
            "Enable this to have all inputs interpreted w.r.t. the local frame.", false, false, false);
	tJoystickUseDeadManSwitch = addOption("tJoystickUseDeadManSwitch",
            "Disable this to fly without pressing a special button simultaneously.", true, false, false);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	// no Parameters
	//parameterInitialized = true;
}

void Joystick::destroy()
{

}

bool Joystick::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	joySub = nodeHandle.subscribe(tJoystickTopic->getValue()
			, 10, &Joystick::joystickCB, this);

	lastVelocityInput = Velocity3D::Zero();
	lastYawRateInput = 0.0;

	if (tJoystickUsePositionMode->getValue()) {
		posModeCurPosition = currentState.position;
		posModeCurYawAngle = currentState.getEulerRPY()(2);
		posModeLastInputTime = Time();
	}

	valid = true;

	return true;
}

void Joystick::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void Joystick::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void Joystick::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	joySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void Joystick::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity(lastVelocityInput);
	generatedTrajInput.setYawRate(lastYawRateInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Joystick::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	//	ROS_INFO_STREAM("InputVel: " << std::endl << lastVelocityInput);
	//	ROS_INFO_STREAM("InputYawRate: " << lastYawRateInput);

		// set Velocity Mode
		if (tJoystickUsePositionMode->getValue()) {
// 			std::cout << "P" << std::endl;
			// integrate Position
			double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
	//		ROS_INFO("timeDiffSec: %f", timeDiffSec);
			posModeLastInputTime = Time();
			if (tJoystickUseRelativeMode->getValue()) {
				//all is w.r.t. local frame
				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
				posModeCurPosition += (newVel * timeDiffSec);
			} else {
				//all is w.r.t. world frame
				posModeCurPosition += (lastVelocityInput * timeDiffSec);
			}

			posModeCurYawAngle += (lastYawRateInput * timeDiffSec);

	//		ROS_INFO_STREAM("Position: " << std::endl << posModeCurPosition);
	//		ROS_INFO_STREAM("YawAngle: " << posModeCurYawAngle.dCast());

			if (tJoystickUseRelativeMode->getValue()) {
				//all is w.r.t. local frame
				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
				generatedTrajInput.setPosition(posModeCurPosition,newVel);
				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);
			} else {
				//all is w.r.t. world frame
				generatedTrajInput.setPosition(posModeCurPosition, lastVelocityInput);
				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);
				
			}

		} else {
// 			std::cout << "V" ;
			if (tJoystickUseRelativeMode->getValue()) {
// 				std::cout << "1" ;
				//all is w.r.t. local frame
				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
				generatedTrajInput.setVelocity(newVel);
				generatedTrajInput.setYawRate(lastYawRateInput);
			} else{
// 				std::cout << "2" ;
				//all is w.r.t. world frame
				generatedTrajInput.setVelocity(lastVelocityInput);
				generatedTrajInput.setYawRate(lastYawRateInput);// ### RE-ENABLE THIS
// 				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput); // ### ELIMINATE THIS!!!!!!!
			}
// 			std::cout << std::endl;
		}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void Joystick::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Joystick::isValid(const TKState& currentState) const
{
	// never turns invalid
	return valid;
}


}
