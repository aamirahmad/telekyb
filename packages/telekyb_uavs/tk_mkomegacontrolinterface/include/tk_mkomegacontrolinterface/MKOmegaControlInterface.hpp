/*
 * MKOmegaControlInterface.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef MKOMEGACONTROLINTERFACE_HPP_
#define MKOMEGACONTROLINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceOptions.hpp>
#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceConnection.hpp>

#include <tk_mkinterface/MKData.hpp>
#include <tk_mkomegacontrolinterface/MKOmegaControlROSInterface.hpp>
#include <tk_mkomegacontrolinterface/MKOmegaControlSafeMod.hpp>

#include <telekyb_base/Filter/IIRFilter.hpp>

namespace TELEKYB_NAMESPACE {

/**
 * Note: Always check that hasConnection returns true after Initialization
 */

class MKOmegaControlInterface : public MKSafeModDeleagte {
protected:
	MKOmegaControlInterfaceOptions options;

	// MKSafeMod
	MKOmegaControlSafeMod* safeModule;

	// Connection
	MKOmegaControlInterfaceConnection* connection;

	// ROS Interface
	MKOmegaControlROSInterface* rosInterface;

	double battery;

	bool receivedFirstBatteryLevel;
	
	bool performIMUCalibration();
	
	/// \todo TODO IMPLEMENT ME
	bool performAccelerometerCalibration(); // TODO implement this method

	Myfoo_IMU offset;

public:
	MKOmegaControlInterface();
	virtual ~MKOmegaControlInterface();

	bool hasConnection() const;
	
	// these methods are empty shells and through an ecception
	void handleCommand(double estMass, double pitch, double roll, double yawrate, double thrust);
	void handleCommand(const std::vector<short int, std::allocator<short int> > & spCommands);
	
	// this is the only handleCommand method which is actually used
	void handleCommand(const std::vector<double, std::allocator<double> > & blCommands);
	
	MKOmegaControlInterfaceConnection* getConnection() const;
	
	// this method performs onboard gyro drift estimation
	bool performDriftEstim();
	
	// MKSafeModListener
	void safeModDidBecomeActive();
	void safeModFinished();

	// Called by ROS Interface Timer. Emits Warning when Low. returns true if empty!
	bool checkBattery(MKInt& batteryValue, bool& landRequest);
	void setEmergency();

	const MKOmegaControlInterfaceOptions& getOptions() const;

};

}

#endif /* MKINTERFACE_HPP_ */
