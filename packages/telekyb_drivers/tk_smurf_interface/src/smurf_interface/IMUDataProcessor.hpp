/*
 * IMUDataProcessor.hpp
 *
 *  Created on: Aug 16, 2012
 *      Author: mriedel
 */

#ifndef IMUDATAPROCESSOR_HPP_
#define IMUDATAPROCESSOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_base/Time.hpp>

#include "SerialIMUDevice.hpp"
#include "IMUDataProcessorOptions.hpp"
#include "IMUCalibrator.hpp"

// IMU Message
#include <tk_draft_msgs/TKSmallImu.h>
#include <geometry_msgs/Wrench.h>

namespace TELEKYB_NAMESPACE {

class IMUDataProcessor : public RawImuDataListener {
private:
	IMUDataProcessorOptions options;

	IMUCalibrator calibrator;

	// Publisher
	ros::Publisher imuPublisher;
	ros::Publisher imuPublisherAsWrenchMsg;

	bool driftEstimRunning;

	// predefined msg to not place it on the stack all the time.
	tk_draft_msgs::TKSmallImu rosMsg;

public:
	IMUDataProcessor();
	virtual ~IMUDataProcessor();

	bool init();

	void processRawIMUData(const RawImuData& data);
	void attitudeEstimation(const RawImuData& data);
};

} /* namespace telekyb */
#endif /* IMUDATAPROCESSOR_HPP_ */
