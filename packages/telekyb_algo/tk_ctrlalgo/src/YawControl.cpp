/*
 * YawControl.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include <tk_ctrlalgo/YawControl.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

#include <ros/console.h>
namespace TELEKYB_NAMESPACE {


// Options
YawControlOptions::YawControlOptions()
	: OptionContainer("YawControl")
{
	tPropGain = addOption<double>( "tPropGain",
			"Yaw Proportional gain for control with onboard yaw rate measure", 5.0, false, false);
	tDerivGain = addOption<double>( "tDerivGain",
			"Yaw Derivative gain for control with onboard yaw rate measurement", 1.0, false, false);
	tPropGainExt = addOption<double>( "tPropGainExt",
			"Yaw Proportional gain for control with external yaw rate measurement", 5.0, false, false);
	tDerivGainExt = addOption<double>( "tDerivGainExt",
			"Yaw Derivative gain for control with external yaw rate measurement", 15.0, false, false);
}

YawControl::YawControl()
{
}

YawControl::~YawControl()
{

}

void YawControl::run(const TKTrajectory& input, const TKState& currentState, YawCtrlOutput& output)
{

    /*
     *  set logger level permantly to debug
    */
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
       ros::console::notifyLoggerLevelsChanged();
    }


    double yawAngleErr = Angle::normPi(input.yawAngle - currentState.getEulerRPY()(2)); // desYawAngle - currentYawAngle
	//double yawAngleErr = sin(input.yawAngle - currentState.getEulerRPY()(2)); // desYawAngle - currentYawAngle
	double yawRateErr = input.yawRate - currentState.angVelocity(2); // desYawRate - currentYawRate
	


	
	double kp = 0.0, kd = 1.0, ka = 0.0;
// 	Decimal kp_vel=getYawVelPropGain();;
	if (input.yawCtrl == YawControlType::AngleOnBoard) {
		kp = options.tPropGain->getValue();
		kd = options.tDerivGain->getValue();
// 		std::cout << "C1" << std::endl;
		ka = 0.0;
		
			output.comYaw = ka*input.yawAcceleration + kd*yawRateErr + kp*yawAngleErr; // desired Yaw
		
		
		
	} else if (input.yawCtrl == YawControlType::RateOnBoard) {
// 		std::cout << "D1" << std::endl;
//		kp = 0.0;
//		kd = 1.0;
//		ka = 0.0;


	output.comYaw = input.yawRate; // desired YawRate


	} else if (input.yawCtrl == YawControlType::AngleOffBoard) {
// 		std::cout << "E1" << std::endl;
		kp = options.tPropGainExt->getValue();
		kd = options.tDerivGainExt->getValue();
		ka = 1.0;
		
		
			output.comYaw = ka*input.yawAcceleration + kd*yawRateErr + kp*yawAngleErr; // desired Yaw
		
	} else if (input.yawCtrl == YawControlType::RateOffBoard) {
// 		std::cout << "F1" << std::endl;
//		kp = 0.0;
		kd = options.tDerivGainExt->getValue();
		ka = 1.0;
		
			output.comYaw = ka*input.yawAcceleration + kd*yawRateErr + kp*yawAngleErr; // desired Yaw
		
	} else if (input.yawCtrl == YawControlType::AccelerationOnBoard) {
		ROS_ERROR("YawControlType::AccelerationOnBoard not implemented!");
		
		
			output.comYaw = ka*input.yawAcceleration + kd*yawRateErr + kp*yawAngleErr; // desired Yaw
		
	} else if (input.yawCtrl == YawControlType::AccelerationOffBoard) {
		ROS_ERROR("YawControlType::AccelerationOffBoard not implemented!");
		
			output.comYaw = ka*input.yawAcceleration + kd*yawRateErr + kp*yawAngleErr; // desired Yaw
		
	}
	
    //output.comYaw = 0.0;
    ROS_DEBUG_STREAM_THROTTLE(1,"tk_ctrlAlgo::YawControl comYaw " << output.comYaw);

//	Angle yawErr = Angle(in->desiredYaw - in->currentYaw);


	// DEBUG
    ROS_DEBUG_STREAM_THROTTLE(1,"tk_ctrlAlgo::YawControl" << " input.yawAngle: " << input.yawAngle);
    ROS_DEBUG_STREAM_THROTTLE(1,"tk_ctrlAlgo::YawControl" << " current yawAngle: " << currentState.getEulerRPY()(2));
    ROS_DEBUG_STREAM_THROTTLE(1,"tk_ctrlAlgo::YawControl" << " yawAngleError: " << yawAngleErr);
    ROS_DEBUG_STREAM_THROTTLE(1,"tk_ctrlAlgo::YawControl" << " yawRateErr: " << yawRateErr);
    ROS_DEBUG_THROTTLE(1,"tk_ctrlAlgo::YawControl Angle::normPi(yawAngleErr): %f", Angle::normPi(yawAngleErr));
	//output.comYaw = ka*input.desYawAcceleration + kd*yawRateErr + kp*yawErr.dCastPi();

	//output.comYaw = -output.comYaw;
}

}
