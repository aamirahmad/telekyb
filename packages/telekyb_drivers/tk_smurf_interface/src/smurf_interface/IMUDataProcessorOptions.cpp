/*
 * IMUDataProcessorOptions.cpp
 *
 *  Created on: Aug 23, 2012
 *      Author: mriedel
 */

#include "IMUDataProcessorOptions.hpp"


#include <telekyb_defines/physic_defines.hpp>

namespace TELEKYB_NAMESPACE {

IMUDataProcessorOptions::IMUDataProcessorOptions()
 : OptionContainer("IMUDataProcessor")
{

	tInitialDriftEstim = addOption<bool>("tInitialDriftEstim",
				"Do Drift Estimation before at Start-Up", true, false, true);

	tAccOffsets = addOption<Eigen::Vector3d>("tAccOffsets",
			"Accelerometer Offset Values (x,y,z)", Eigen::Vector3d::Zero(), false, false);
	tGyroOffsets = addOption<Eigen::Vector3d>("tGyroOffsets",
			"Gyroscope Offset Values (roll,pitch,yaw)", Eigen::Vector3d::Zero(), false, false);

	tAccScale = addOption<double>("tAccScale",
			"Accelerometer Scale over the range 0-1024", 5.0 * GRAVITY , false, true);

	tGyroScale = addOption<double>("tGyroScale",
			"Gyro Scale over the range 0-1024", 600.0/180.0*M_PI, false, true);//600 should be ~900?

	// Topic Name
	tTopicName = addOption< std::string >("tTopicName",
			"Topic to publish Imu Message to...",
			"imuData" , false, true);
	tPublishRaw = addOption<bool>("tPublishRaw",
			"Publish Raw QC Data", true, false, true);
	tPublishIMUAsWrenchMsg = addOption<bool>("tPublishIMUAsWrenchMsg",
		"Publish lin acceleration and ang Velocity as a geometry_msgs/Wrench", false, false, true);
		// Topic Name
	tIMUAsWrenchTopicName = addOption< std::string >("tIMUAsWrenchTopicName",
			"Topic to publish Imu as geometry_msgs/Wrench Message to...",
			"imuDataAsWrench" , false, true);
	
	tIMUReferenceFrame = addOption< int >("tIMUReferenceFrame",
			"Frame of the published IMU: 0 is NED, 1 is NWU",
			0 , false, true);
}

} /* namespace telekyb */
