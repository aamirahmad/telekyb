/*
 * MKSafeMod.hpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#ifndef MKSAFEMOD_HPP_
#define MKSAFEMOD_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Options.hpp>

#include <boost/thread.hpp>

#include <tk_mkinterface_outdoor/MKInterfaceConnection.hpp>

namespace TELEKYB_NAMESPACE {

class MKSafeModOptions : public OptionContainer
{
public:
	Option<int>* tCmdTimeoutUs;
	Option<int>* tEmergLandThrust;
	Option<double>* tEmergLandDuration;
	Option<int>* tEmergLandFreq;

//	Option<int>* tThreadSleepTimeUs;
	MKSafeModOptions();
};

class MKSafeModDeleagte
{
public:
	virtual ~MKSafeModDeleagte() {}
	virtual void safeModDidBecomeActive() = 0;
	virtual void safeModFinished() = 0;
	virtual void handleCommand(double estMass, double roll, double pitch, double thrust, double yaw) = 0;
};

class MKSafeMod {
protected:
	MKSafeModOptions options;
	Timer cmdTimer; // Reset with each command;
	MKInterfaceConnection* connection;

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
	MKSafeMod(MKSafeModDeleagte* delegate_, MKInterfaceConnection* connection_);
	virtual ~MKSafeMod();

	void start();
	void stop();

	// returns true if SafeMod has Taken Over
	bool isActive() const;
	bool isRunning() const;

	void resetCmdTimer();

	void setEmergency();
};

} /* namespace telekyb */
#endif /* MKSAFEMOD_HPP_ */
