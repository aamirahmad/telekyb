/*
 * MKInterface.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef MKINTERFACE_HPP_
#define MKINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_mkinterface/MKInterfaceOptions.hpp>
#include <tk_mkinterface/MKData.hpp>

#include <tk_mkinterface/MKInterfaceConnection.hpp>

#include <tk_mkinterface/MKROSInterface.hpp>

#include <tk_mkinterface/MKSafeMod.hpp>

#include <telekyb_base/Filter/IIRFilter.hpp>

namespace TELEKYB_NAMESPACE {

/**
 * Note: Always check that hasConnection returns true after Initialization
 */

class MKInterface : public MKSafeModDeleagte {
protected:
	MKInterfaceOptions options;

	// MKSafeMod
	MKSafeMod* safeModule;

	// Connection
	MKInterfaceConnection* connection;

	// ROS Interface
	MKROSInterface* rosInterface;

	IIRFilter* batteryFilter;
	MKValue* battery;

	bool receivedFirstBatteryLevel;


public:
	MKInterface();
	virtual ~MKInterface();

	bool hasConnection() const;
	void handleCommand(double estMass, double pitch, double roll, double yawrate, double thrust);
	void handleCommand(const std::vector<double, std::allocator<double> > & blCommands);
	void handleCommand(const std::vector<short int, std::allocator<short int> > & spCommands);
	
	MKUChar blSetpoint(double force) const;
	MKUChar blSetpoint(double force, double battery) const;

	MKInterfaceConnection* getConnection() const;

	bool performDriftEstim();

	// MKSafeModListener
	void safeModDidBecomeActive();
	void safeModFinished();

	// Called by ROS Interface Timer. Emits Warning when Low. returns true if empty!
	bool checkBattery(MKInt& batteryValue, bool& landRequest);
	void setEmergency();

	const MKInterfaceOptions& getOptions() const;

	double batteryFiltered;

};

}

#endif /* MKINTERFACE_HPP_ */
