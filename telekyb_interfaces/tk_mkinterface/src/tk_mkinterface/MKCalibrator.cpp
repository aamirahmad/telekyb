/*
 * MKCalibrator.cpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKCalibrator.hpp>

#define DRIFT_ESTIM_WINDOW_SIZE 50

namespace TELEKYB_NAMESPACE {

MKCalibratorOptions::MKCalibratorOptions()
	: OptionContainer("MKCalibrator")
{
	tDriftEstimTimeout = addBoundsOption<double>("tDriftEstimTimeout",
			"Timeout for DriftEstim in seconds", 10.0, 0.1, 30.0, false, true);
	tDriftEstimDeltaThreshold = addBoundsOption<int>("tDriftEstimDeltaThreshold",
			"Threshold for Drift Estimation Deltas", 160, 1, 500, false, false);
	tDriftEstimDataPeriod = addBoundsOption<int>("tDriftEstimDataPeriod",
			"Period of time to request data during the drift estimation", 1, 1, 500, false, false);
}

MKCalibrator::MKCalibrator(MKInterfaceConnection* connection_)
	: connection(connection_), gyroDriftEstimRunning(false)
{

}

MKCalibrator::~MKCalibrator()
{
 // not needed
}

// blocking DriftEstim
bool MKCalibrator::doGyroDriftEstim()
{
	ROS_INFO("Starting Drift Estimation!");
	// get certain values
	MKValue* timeMirrorPeriod = connection->getMKDataRef().getValueByID(MKDataDefines::MIRROR_TIME_PERIOD);
	MKValue* motorState = connection->getMKDataRef().getValueByID(MKDataDefines::MOTOR_STATE);
	MKValue* driftEstimActive = connection->getMKDataRef().getValueByID(MKDataDefines::DRIFT_ESTIM_ACTIVE);
	MKValue* mirrorDataActive = connection->getMKDataRef().getValueByID(MKDataDefines::MIRROR_DATA_ACTIVE);

	if (!(connection->updateValue(timeMirrorPeriod->getID()) &&
			connection->updateValue(motorState->getID()) &&
			connection->updateValue(driftEstimActive->getID()) &&
			connection->updateValue(mirrorDataActive->getID())) ) {
		ROS_ERROR("Could not update values. Drift not successful.");
		return false;
	}

	// Check
	if (motorState->getValue() != MotorState::Off) {
		MotorState currMotorState(motorState->getValue());
		ROS_ERROR("Motor State must be off for Drift Estimation! Current: %s" ,currMotorState.str());
		return false;
	}

	// ok
	if (driftEstimActive->getValue() != MKDataDefines::MKINT_LOGICAL_OFF) {
		ROS_WARN("Drift Estimation is unequal to MKINT_LOGICAL_OFF. Unexpected State to start..");
	}

	// get Initial Drift Values;
	MKValue* driftGyroX = connection->getMKDataRef().getValueByID(MKDataDefines::DRIFT_GYRO_X);
	MKValue* driftGyroY = connection->getMKDataRef().getValueByID(MKDataDefines::DRIFT_GYRO_Y);
	MKValue* driftGyroZ = connection->getMKDataRef().getValueByID(MKDataDefines::DRIFT_GYRO_Z);
	if (!(	connection->updateValue(driftGyroX->getID()) &&
			connection->updateValue(driftGyroY->getID()) &&
			connection->updateValue(driftGyroZ->getID()) )) {

		ROS_ERROR("Unable to get initial Gyro Drift States for Drift Estimation!");
		return false;
	}

	// Set initial Values
	minDrifts.x = driftGyroX->getValue();
	minDrifts.y = driftGyroY->getValue();
	minDrifts.z = driftGyroZ->getValue();
	maxDrifts = minDrifts;

	// Null Init
	driftCounters.x = 0;
	driftCounters.y = 0;
	driftCounters.z = 0;
	driftEstimDone = driftCounters;


	// save
	MKInt saveTimeMirrorPeriod = timeMirrorPeriod->getValue();
	MKInt saveMirrorDataActive = mirrorDataActive->getValue();
	MKActiveIDs saveActiveIDs = connection->getActiveDataIDs();

	// START ACTUAL ESTIMATION
	// no checking for success.
	connection->setActiveDataIDs(MKData::getPattern(MKDataPattern::AccOffsetGyroDrift));
	connection->setValue(timeMirrorPeriod->getMKSingleValuePacketWithValue(options.tDriftEstimDataPeriod->getValue())); // highest rate
	connection->setValue(mirrorDataActive->getMKSingleValuePacketWithValue(MKDataDefines::MKINT_LOGICAL_ON));
	connection->setValue(driftEstimActive->getMKSingleValuePacketWithValue(MKDataDefines::MKINT_LOGICAL_ON));

	// register
	connection->registerMKDataListener(this);

	Timer timeoutTimer;

	// loop
	bool returnValue = false;

	while(timeoutTimer.getElapsed().toDSec() < options.tDriftEstimTimeout->getValue()) {

		if (driftEstimDone.x && driftEstimDone.y && driftEstimDone.z) {
			// all done
			//ROS_INFO("Successfully did driftEstimation!");
			returnValue = true;
			break;
		}


		usleep(1000);
	}

	// done estimating
	connection->unRegisterMKDataListener(this);

	// set back
	connection->setValue(driftEstimActive->getMKSingleValuePacketWithValue(MKDataDefines::MKINT_LOGICAL_OFF));
	connection->setValue(mirrorDataActive->getMKSingleValuePacketWithValue(saveMirrorDataActive));
	connection->setValue(timeMirrorPeriod->getMKSingleValuePacketWithValue(saveTimeMirrorPeriod));
	connection->setActiveDataIDs(saveActiveIDs);

	if (returnValue) {
		ROS_INFO("Drift Estimation was successful!");
	} else {
		ROS_ERROR("Drift Estimation was not done successfully.");
	}
	return returnValue;
}

void MKCalibrator::dataValueUpdated(MKValue* value)
{
	// we are only interested in the drift values.
	switch (value->getID()) {
		case MKDataDefines::DRIFT_GYRO_X:
			//ROS_INFO("Received value DRIFT_GYRO_X = %d", value->getValue());

			// already done.
						if (driftEstimDone.x != 0) {
				break;
			}

			if (driftCounters.x == DRIFT_ESTIM_WINDOW_SIZE) {
				ROS_INFO("Drift Error X: %d", maxDrifts.x - minDrifts.x);
// 				ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
// 				std::cout << options.tDriftEstimDeltaThreshold->getValue() << std::endl;
				if (maxDrifts.x - minDrifts.x < options.tDriftEstimDeltaThreshold->getValue() && maxDrifts.x-minDrifts.x!=0) {
					driftEstimDone.x = 1;
					ROS_INFO("Drift Estimation X done!");
				}

				minDrifts.x = value->getValue();
				maxDrifts.x = value->getValue();
				driftCounters.x = 0;
			}

			minDrifts.x = std::min(minDrifts.x, (int)value->getValue());
			maxDrifts.x = std::max(maxDrifts.x, (int)value->getValue());
			driftCounters.x++;

			break;
		case MKDataDefines::DRIFT_GYRO_Y:
			//ROS_INFO("Received value DRIFT_GYRO_Y = %d", value->getValue());

			// already done.
			if (driftEstimDone.y != 0) {
				break;
			}

			if (driftCounters.y == DRIFT_ESTIM_WINDOW_SIZE) {
				ROS_INFO("Drift Error Y: %d", maxDrifts.y - minDrifts.y);
				if (maxDrifts.y - minDrifts.y < options.tDriftEstimDeltaThreshold->getValue() && maxDrifts.y-minDrifts.y!=0) {
					driftEstimDone.y = 1;
					ROS_INFO("Drift Estimation Y done!");
				}

				minDrifts.y = value->getValue();
				maxDrifts.y = value->getValue();
				driftCounters.y = 0;
			}

			minDrifts.y = std::min(minDrifts.y, (int)value->getValue());
			maxDrifts.y = std::max(maxDrifts.y, (int)value->getValue());
			driftCounters.y++;

			break;
		case MKDataDefines::DRIFT_GYRO_Z:
			//ROS_INFO("Received value DRIFT_GYRO_Z = %d", value->getValue());

			// already done.
			if (driftEstimDone.z != 0) {
				break;
			}

			if (driftCounters.z == DRIFT_ESTIM_WINDOW_SIZE) {
				ROS_INFO("Drift Error Z: %d", maxDrifts.z - minDrifts.z);
				if (maxDrifts.z - minDrifts.z < options.tDriftEstimDeltaThreshold->getValue() && maxDrifts.z-minDrifts.z!=0) {
					driftEstimDone.z = 1;
					ROS_INFO("Drift Estimation Z done!");
				}

				minDrifts.z = value->getValue();
				maxDrifts.z = value->getValue();
				driftCounters.z = 0;
			}

			minDrifts.z = std::min(minDrifts.z, (int)value->getValue());
			maxDrifts.z = std::max(maxDrifts.z, (int)value->getValue());
			driftCounters.z++;

			break;

		default:
			// we don't care about other values
			break;
	}

}

} /* namespace telekyb */
