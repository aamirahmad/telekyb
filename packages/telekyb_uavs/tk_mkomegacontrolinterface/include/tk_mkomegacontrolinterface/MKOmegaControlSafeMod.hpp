/*
 * MKOmegaControlSafeMod.hpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#ifndef MKOMEGACONTROLSAFEMOD_HPP_
#define MKOMEGACONTROLSAFEMOD_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Options.hpp>

#include <boost/thread.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceConnection.hpp>
#include <tk_mkinterface/MKSafeMod.hpp>

namespace TELEKYB_NAMESPACE {

class MKOmegaControlSafeModOptions : public OptionContainer
{
public:
	Option<int>* tCmdTimeoutUs;
	Option<int>* tEmergLandThrust;
	Option<double>* tEmergLandDuration;
	Option<int>* tEmergLandFreq;

//	Option<int>* tThreadSleepTimeUs;
	MKOmegaControlSafeModOptions();
};

// class MKOmegaControlSafeModDeleagte
// {
// public:
// 	virtual ~MKOmegaControlSafeModDeleagte() {}
// 	virtual void safeModDidBecomeActive() = 0;
// 	virtual void safeModFinished() = 0;
// 	virtual void handleCommand(double estMass, double roll, double pitch, double thrust, double yaw) = 0;
// };

class MKOmegaControlSafeMod {
protected:
	MKOmegaControlSafeModOptions options;
	Timer cmdTimer; // Reset with each command;
	MKOmegaControlInterfaceConnection* connection;

	bool active;

	MKSafeModDeleagte* delegate;

	// generates Cmds.
	void safeModFcn();

	// Independent Threading
	boost::thread* mkSafeModThread;

	bool threadStopRequest;
	bool emergencyLandingRequest;
	//void initReadingThread();
	void threadFcn();
	void startThread();
	void stopThread();

public:
	MKOmegaControlSafeMod(MKSafeModDeleagte* delegate_, MKOmegaControlInterfaceConnection* connection_);
	virtual ~MKOmegaControlSafeMod();

	void start();
	void stop();

	// returns true if SafeMod has Taken Over
	bool isActive() const;
	bool isRunning() const;

	void resetCmdTimer();

	void setEmergency();
};

} /* namespace telekyb */
#endif /* MKOMEGACONTROLSAFEMOD_HPP_ */
