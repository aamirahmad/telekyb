/*
 * MKOmegaControlCalibrator.hpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#ifndef MKOMEGACONTROLCALIBRATOR_HPP_
#define MKOMEGACONTROLCALIBRATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_mkinterface/MKData.hpp>
#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceConnection.hpp>

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

class MKOmegaControlCalibratorOptions : public OptionContainer {
public:
	Option<double>* tDriftEstimTimeout;
	Option<int>* tDriftEstimDeltaThreshold;
	Option<int>* tDriftEstimDataPeriod;
	MKOmegaControlCalibratorOptions();
};

class MKOmegaControlCalibrator {
protected:
// 	MKOmegaControlCalibratorOptions options;
// 	MKOmegaControlInterfaceConnection* connection;
// 	bool gyroDriftEstimRunning;
// 
// 	// Algorithm Variables
// 	GyroDrifts driftCounters; // counters for x,y,z
// 	GyroDrifts driftEstimDone; // 0: not done. other: done!
// 
// 	GyroDrifts minDrifts;
// 	GyroDrifts maxDrifts;

public:
	MKOmegaControlCalibrator(MKOmegaControlInterfaceConnection* connection_);
	virtual ~MKOmegaControlCalibrator();

// 	bool doGyroDriftEstim();

	// Listener CB
// 	void dataValueUpdated(MKValue* value);
};

} /* namespace telekyb */
#endif /* MKOMEGACONTROLCALIBRATOR_HPP_ */
