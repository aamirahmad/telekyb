/*
 * TeleopPointBehavior.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include "TeleopPoint.hpp"
#include <telekyb_base/ROS.hpp>

#define MAX_INITIAL_VELOCITY 0.05


// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::TeleopPoint, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

enum COMMAND_MAPPING{
    COMMAND_THRUST = 1,
    COMMAND_LEFTRIGHT = 3,
    COMMAND_FORWARDBACK = 4,
    COMMAND_YAW = 0,
    COMMAND_DISABLE = 5 // Logitech Joypad RB
};

TeleopPoint::TeleopPoint()
    : Behavior("tk_behavior/TeleopPoint",  BehaviorType::Air),
      nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
      tJoystickTopic(NULL)
{
    valid = false;
    _joyMsgCalledbackOnce = false;
}

void TeleopPoint::initialize()
{
    /*
     *  set logger level permantly to debug
     */
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    tJoystickTopic = addOption<std::string>("tJoystickTopic","JoystickTopic that published sensor_msgs::Joy","/TeleKyb/tJoy/joy", false, false);

    // the Standard Brake Minimal Value should be significantly below this value!
    tTeleopPointMaxInitialVelocity = addOption<double>("tTeleopPointMaxInitialVelocity",
                                                       "Defines the Maximal Initial Velocity to be able to switch into TeleopPoint",
                                                       0.2, false, true);
    tTeleopPointVelocityMode = addOption<bool>("tTeleopPointVelocityMode",
                                               "TeleopPoint in velocity mode",
                                               false, false, false);

    tTeleopPointStartingPosition = addOption<Position3D>("tTeleopPointStartingPosition", "Start the joypad teleoperationat certain point",
                                                         Position3D(0.0, 0.0, -1.0), false, false);

    tTeleopPointScalingThrust = addOption<double>("tTeleopPointScalingThrust","Scaling factor for the thrust or z Axis input",0.1, false, false);
    tTeleopPointScalingRoll = addOption<double>("tTeleopPointScalingRoll","Scaling factor for the y Axis input",0.1, false, false);
    tTeleopPointScalingPitch = addOption<double>("tTeleopPointScalingPitch","Scaling factor for the x Axis input",0.1, false, false);
    tTeleopPointScalingYaw = addOption<double>("tTeleopPointScalingYaw","Scaling the yaw input",0.01, false, false);


    tTeleopPointScalingZ = addOption<double>("tTeleopPointScalingZ","Scaling factor for the thrust or z Axis input",0.1, false, false);
    tTeleopPointScalingY = addOption<double>("tTeleopPointScalingY","Scaling factor for the y Axis input",0.1, false, false);
    tTeleopPointScalingX = addOption<double>("tTeleopPointScalingX","Scaling factor for the x Axis input",0.1, false, false);
    tTeleopPointScalingYawDot = addOption<double>("tTeleopPointScalingYawDot","Scaling the yaw input",0.01, false, false);

    //options = new TeleopPointOptions(this);
    // can work with default parameters
    parameterInitialized = true;
    _velocity = Velocity3D::Zero();
    _yawRate = 0.0;
}

void TeleopPoint::destroy()
{

}
void TeleopPoint::joystickCB(const sensor_msgs::Joy::ConstPtr& msg) {


    if (msg->axes.size() < 4 && msg->buttons.size() < 2) {
        ROS_ERROR("We need at least 4 Axes and at least 2 Buttons");
        return;
    }
    //	ROS_INFO("Received Joystick CB");
    _joyMsg = *msg;
    _joyMsgCalledbackOnce = true;
    // invalidate
    if ( msg->buttons[COMMAND_DISABLE] ) {
        valid = false;
    }
}

bool TeleopPoint::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
    _joySub = nodeHandle.subscribe(tJoystickTopic->getValue(), 10, &TeleopPoint::joystickCB, this);
    // Error if too fast ? Put as parameter?
    if (currentState.linVelocity.norm() > tTeleopPointMaxInitialVelocity->getValue()) {
        ROS_ERROR_STREAM("Too fast to switch into TeleopPoint! Current Velocity Norm: " << currentState.linVelocity.norm());
        return false;
    }

    _position = tTeleopPointStartingPosition->getValue();
    _yawAngle = currentState.getEulerRPY()(2);

    // from Martin:
    //ROS_INFO("willBecomeActive");
    //ROS_INFO_STREAM("Setting Position: " << std::endl << currentState.position << " Angle: " << currentState.getEulerRPY()(2));
    //ROS_INFO("TeleopPoint: Storing yawAngle: %f ", _yawAngle);

    // Thomas: changed to one line output
    valid = true;

    return true;
}

void TeleopPoint::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
    // not used
    //	ROS_ERROR("Sleeping after Switch to TeleopPoint.");
    //	usleep(2*1000*1000); // 10 sec
}

void TeleopPoint::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
    // not used
}

void TeleopPoint::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
    _joySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void TeleopPoint::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    // Same as active
    trajectoryStepActive(currentState,generatedTrajInput);
}

void TeleopPoint::updateVelocity()
{
    if(_joyMsgCalledbackOnce){
        try{

            _yawRate = -_joyMsg.axes[COMMAND_YAW]*tTeleopPointScalingYawDot->getValue();


            _velocity(0) = _joyMsg.axes[COMMAND_FORWARDBACK]*tTeleopPointScalingX->getValue();
            _velocity(1) = -_joyMsg.axes[COMMAND_LEFTRIGHT]*tTeleopPointScalingY->getValue();

            _velocity(2) = _joyMsg.axes[COMMAND_THRUST]*-1*tTeleopPointScalingZ->getValue();


        }catch(...){
            ROS_ERROR_STREAM("tk_be_common/TeleopPoint :: happend while updating the Velocity");
        }
    }

}

void TeleopPoint::updatePosition(){
    if(_joyMsgCalledbackOnce){
        try{

            _yawAngle += -_joyMsg.axes[COMMAND_YAW]*tTeleopPointScalingYaw->getValue();
            if(_yawAngle > M_PI){ _yawAngle = -M_PI + (_yawAngle - M_PI); }
            if(_yawAngle < -M_PI){ _yawAngle = M_PI - (_yawAngle + M_PI); }


            double x = _joyMsg.axes[COMMAND_FORWARDBACK]*tTeleopPointScalingPitch->getValue();
            double y = -_joyMsg.axes[COMMAND_LEFTRIGHT]*tTeleopPointScalingRoll->getValue();

            _position(0) += x * cos(_yawAngle) - y * sin(_yawAngle) ;
            _position(1) += y * cos(_yawAngle) + x * sin(_yawAngle);

            _position(2) += _joyMsg.axes[COMMAND_THRUST]*-1*tTeleopPointScalingThrust->getValue();


        }catch(...){
            ROS_ERROR_STREAM("tk_be_common/TeleopPoint :: happend while updating the Position");
        }
    }
}

void TeleopPoint::setTrajectoryHeader(TKTrajectory& generatedTrajInput){
    generatedTrajInput.setHeader(ros::Time::now(),"/redfox_1/base_link");
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or TeleopPoint if undef).
void TeleopPoint::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{

    if (tTeleopPointVelocityMode->getValue()) {
        this->updateVelocity();
        generatedTrajInput.setVelocity(_velocity);
        generatedTrajInput.setYawRate(_yawRate);
    } else {
        this->updatePosition();
        generatedTrajInput.setPosition(_position);
        generatedTrajInput.setYawAngle(_yawAngle);
    }
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void TeleopPoint::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    // Same as active
    generatedTrajInput.setHeader(ros::Time::now(),"");
    trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool TeleopPoint::isValid(const TKState& currentState) const
{
    // never turns invalid
    return true;
}



}
