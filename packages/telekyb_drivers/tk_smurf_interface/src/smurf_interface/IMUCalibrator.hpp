/*
 * IMUCalibrator.hpp
 *
 *  Created on: Aug 23, 2012
 *      Author: mriedel
 */

#ifndef IMUCALIBRATOR_HPP_
#define IMUCALIBRATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

#include "SerialIMUDevice.hpp"

#include <telekyb_base/Time.hpp>

// For statistics
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>


using namespace boost::accumulators;

namespace TELEKYB_NAMESPACE {

class IMUCalibratorOptions : public OptionContainer {
public:
	Option<double>* tDriftEstimTimeout;
	Option<double>* tDriftEstimVarianceThreshold;
	Option<int>* tDriftEstimWindowSize;
	Option<double>* tDriftFilterGain;
	IMUCalibratorOptions();
};

class IMUCalibrator : public RawImuDataListener {
private:
	IMUCalibratorOptions options;

	bool driftEstimationActive;
	bool driftEstimationDone[3];
	Timer driftEstimationTimer;

	Eigen::Vector3d gyroSteadyValues;

	accumulator_set<double, stats<tag::variance(lazy)> > driftEstimAccRoll;
	accumulator_set<double, stats<tag::variance(lazy)> > driftEstimAccPitch;
	accumulator_set<double, stats<tag::variance(lazy)> > driftEstimAccYaw;


public:
	IMUCalibrator();
	virtual ~IMUCalibrator();


	bool driftEstimation(Eigen::Vector3d& gyroOffsets);
	void processRawIMUData(const RawImuData& data);
};

} /* namespace telekyb */
#endif /* IMUCALIBRATOR_HPP_ */
