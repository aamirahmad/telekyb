/*
 * MKCalibrator.hpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#ifndef MKCALIBRATOR_HPP_
#define MKCALIBRATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_mkinterface/MKData.hpp>
#include <tk_mkinterface/MKInterfaceConnection.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

struct GyroDrifts {
	int x;
	int y;
	int z;
	GyroDrifts() {
		x = 0;
		y = 0;
		z = 0;
	}
};

class MKCalibratorOptions : public OptionContainer {
public:
	Option<double>* tDriftEstimTimeout;
	Option<int>* tDriftEstimDeltaThreshold;
	Option<int>* tDriftEstimDataPeriod;
	MKCalibratorOptions();
};

class MKCalibrator : public MKDataListener {
protected:
	MKCalibratorOptions options;
	MKInterfaceConnection* connection;
	bool gyroDriftEstimRunning;

	// Algorithm Variables
	GyroDrifts driftCounters; // counters for x,y,z
	GyroDrifts driftEstimDone; // 0: not done. other: done!

	GyroDrifts minDrifts;
	GyroDrifts maxDrifts;

public:
	MKCalibrator(MKInterfaceConnection* connection_);
	virtual ~MKCalibrator();

	bool doGyroDriftEstim();

	// Listener CB
	void dataValueUpdated(MKValue* value);
};

} /* namespace telekyb */
#endif /* MKCALIBRATOR_HPP_ */
