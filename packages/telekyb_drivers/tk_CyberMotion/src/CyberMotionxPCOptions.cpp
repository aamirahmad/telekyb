/*
 * CyberMotionxPCOptions.cpp
 *
 *  Created on: Sep 5, 2012
 *      Author: Johannes Lächele
 *  
 */

#include "CyberMotionxPCOptions.hpp"

namespace TELEKYB_NAMESPACE {

CyberMotionxPCOptions::CyberMotionxPCOptions()  : OptionContainer("CyberMotionxPCBridge") {
	tdesiredJointStateTopic = addOption< std::string > ("tdesiredJointStateTopic", "Topic on which this will listen for desired joint states", "/TeleKyb/CyberMotion/DesiredJointState", false, true);
	txPCTargetHostName = addOption< std::string > ("txPCTargetHostName", "Defines the host of the xPC Target machine. May also be an IP address", "192.168.34.1", true, true);
	txPCTargetPort = addOption< std::string > ("txPCTargetPort", "Defines the port of the xPC Target machine.", "8885", false, true);

	tLowerPositionLimits = addOption< std::vector<double> >("tLowerPositionLimits", "Lower bound limits of KUKA Axes positions", std::vector<double>(), true, true);
	tUpperPositionLimits = addOption< std::vector<double> >("tUpperPositionLimits", "Upper bound limit of KUKA Axes positions", std::vector<double>(), true, true);

	tLowerVelocityLimits = addOption< std::vector<double> >("tLowerVelocityLimits", "Lower bound limits of KUKA Axes velocities", std::vector<double>(), true, true);
	tUpperVelocityLimits = addOption< std::vector<double> >("tUpperVelocityLimits", "Upper bound limit of KUKA Axes velocities", std::vector<double>(), true, true);

	tLowerAccelerationLimits = addOption< std::vector<double> >("tLowerAccelerationLimits", "Lower bound limits of KUKA Axes accelerations", std::vector<double>(), true, true);
	tUpperAccelerationLimits = addOption< std::vector<double> >("tUpperAccelerationLimits", "Upper bound limit of KUKA Axes accelerations", std::vector<double>(), true, true);

}

} //end namespace TELEKYB_NAMESPACE
