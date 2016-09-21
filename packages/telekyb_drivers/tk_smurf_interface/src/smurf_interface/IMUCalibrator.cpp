/*
 * IMUCalibrator.cpp
 *
 *  Created on: Aug 23, 2012
 *      Author: mriedel
 */

#include "IMUCalibrator.hpp"

#define GYRO_CENTER 512

namespace TELEKYB_NAMESPACE {

IMUCalibratorOptions::IMUCalibratorOptions()
	: OptionContainer("IMUCalibrator")
{
	tDriftEstimTimeout = addBoundsOption<double>("tDriftEstimTimeout",
			"Timeout for DriftEstim in seconds", 10.0, 0.1, 30.0, false, true);
	tDriftEstimVarianceThreshold = addBoundsOption<double>("tDriftEstimVarianceThreshold",
			"Threshold for Drift Estimation Deltas", 1e-3, 1e-5, 20.0, false, false);
	tDriftEstimWindowSize = addBoundsOption<int>("tDriftEstimWindowSize",
			"Window Size for Drift Estimation", 200, 10, 500, false, false);
	tDriftFilterGain = addOption<double>("tDriftFilterGain",
			"Drift Filter Gain for Estimation", 5.0, false, true);
}


IMUCalibrator::IMUCalibrator() {
	driftEstimationDone[0] = false;
	driftEstimationDone[1] = false;
	driftEstimationDone[2] = false;

	driftEstimationActive = false;

}

IMUCalibrator::~IMUCalibrator() {
	// TODO Auto-generated destructor stub
}


bool IMUCalibrator::driftEstimation(Eigen::Vector3d& gyroOffsets) {
	gyroSteadyValues = Eigen::Vector3d::Constant(GYRO_CENTER);
	gyroSteadyValues += gyroOffsets;


	// loop
	bool returnValue = false;


	driftEstimationTimer.reset();
	driftEstimationActive = true;
	Timer timeoutTimer;
	while(timeoutTimer.getElapsed().toDSec() < options.tDriftEstimTimeout->getValue()) {

		if (driftEstimationDone[0] && driftEstimationDone[1] && driftEstimationDone[2]) {
			// all done
			ROS_INFO("Successfully finished driftEstimation!");
			returnValue = true;
			break;
		}


		usleep(1000);
	}

	if (returnValue) {
		gyroOffsets = gyroSteadyValues - Eigen::Vector3d::Constant(GYRO_CENTER);
//		ROS_INFO_STREAM("Calculated Gyro Offsets: " << std::endl << gyroOffsets);
//		sleep(2);
	}

	return returnValue;
}

void IMUCalibrator::processRawIMUData(const RawImuData& data) {
	if (!driftEstimationActive) {
		return;
	}

	double elapsedTime = driftEstimationTimer.getElapsed().toDSec();
	driftEstimationTimer.reset();

	// active
	gyroSteadyValues(0) += options.tDriftFilterGain->getValue()*((double)data.gyroRoll - gyroSteadyValues(0))*elapsedTime;
	gyroSteadyValues(1) += options.tDriftFilterGain->getValue()*((double)data.gyroPitch - gyroSteadyValues(1))*elapsedTime;
	gyroSteadyValues(2) += options.tDriftFilterGain->getValue()*((double)data.gyroYaw - gyroSteadyValues(2))*elapsedTime;

	if (!driftEstimationDone[0]) {
		driftEstimAccRoll( gyroSteadyValues(0) );
	}
	if (!driftEstimationDone[1]) {
		driftEstimAccPitch( gyroSteadyValues(1) );
	}
	if (!driftEstimationDone[2]) {
		driftEstimAccYaw( gyroSteadyValues(2) );
	}

	if ( (signed)count(driftEstimAccRoll) == options.tDriftEstimWindowSize->getValue()) {
		//ROS_INFO("Variance Roll: %f", variance(driftEstimAccRoll));

		if (variance(driftEstimAccRoll) < options.tDriftEstimVarianceThreshold->getValue()) {
			ROS_INFO("Drift Estimation done: Roll = %f", mean(driftEstimAccRoll));
			driftEstimationDone[0] = true;
		}

		// reset
		driftEstimAccRoll = accumulator_set<double, stats<tag::variance(lazy)> >();
	}

	if ( (signed)count(driftEstimAccPitch) == options.tDriftEstimWindowSize->getValue()) {
		//ROS_INFO("Variance Pitch: %f", variance(driftEstimAccPitch));

		if (variance(driftEstimAccPitch) < options.tDriftEstimVarianceThreshold->getValue()) {
			ROS_INFO("Drift Estimation done: Pitch = %f", mean(driftEstimAccPitch));
			driftEstimationDone[1] = true;
		}

		// reset
		driftEstimAccPitch = accumulator_set<double, stats<tag::variance(lazy)> >();
	}

	if ( (signed)count(driftEstimAccYaw) == options.tDriftEstimWindowSize->getValue()) {
		//ROS_INFO("Variance Yaw: %f", variance(driftEstimAccYaw));

		if (variance(driftEstimAccYaw) < options.tDriftEstimVarianceThreshold->getValue()) {
			ROS_INFO("Drift Estimation done: Yaw = %f", mean(driftEstimAccYaw));
			driftEstimationDone[2] = true;
		}

		// reset
		driftEstimAccYaw = accumulator_set<double, stats<tag::variance(lazy)> >();
	}



	//ROS_INFO_STREAM("Vector: " << std::endl << gyroSteadyValues);

}

} /* namespace telekyb */
