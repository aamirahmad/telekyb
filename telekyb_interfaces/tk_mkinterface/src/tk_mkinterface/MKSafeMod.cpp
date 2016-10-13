/*
 * MKSafeMod.cpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKSafeMod.hpp>

#define THREAD_SLEEP_TIME_US 10000 // 10ms

namespace TELEKYB_NAMESPACE {

MKSafeModOptions::MKSafeModOptions()
	: OptionContainer("MKSafeMod")
{
	tCmdTimeoutUs = addBoundsOption<int>("tCmdTimeoutUs",
			"Specifies the Commands Timeout (in us) for the Safe Module to become active.",
			400000, 10000, 500000, false, false); // must be less then a second.
	tEmergLandThrust = addBoundsOption<int>("tEmergLandThrust",
			"LL Emergency Landing Thrust Command To UAV",
			74, 60, 130, false, true);

	tEmergLandDuration = addBoundsOption<double>("tEmergLandDuration",
			"Time spend in Emergency State",
			2.3, 1.0, 10.0, false, true);
	tEmergLandFreq = addBoundsOption<int>("tEmergLandFreq",
			"Frequency of the Landing Commands",
			120, 50, 240, false, true);
//	tThreadSleepTimeUs = addBoundsOption<int>("tThreadSleepTimeUs",
//			"Thread sleep time in us of MKSafeMod",
//			1000, 100, 50000, false, true); // must be less then a second
}

MKSafeMod::MKSafeMod(MKSafeModDeleagte* delegate_, MKInterfaceConnection* connection_)
	: connection(connection_), active(false), delegate(delegate_), mkSafeModThread(NULL), emergencyLandingRequest(false)
{

}

MKSafeMod::~MKSafeMod()
{
	if (mkSafeModThread) {
		stopThread();
	}
}

void MKSafeMod::safeModFcn()
{
	active = true;
	delegate->safeModDidBecomeActive();

	MKCommandsPacket commands;
	commands.thrust = options.tEmergLandThrust->getValue(); // others are 0.

	Timer emergLandTimer;
	Time sleepTimer( 1.0 / (double)options.tEmergLandFreq->getValue() );

	while(emergLandTimer.getElapsed().toDSec() < options.tEmergLandDuration->getValue()) {
		connection->sendCommand(commands);
		sleepTimer.sleep();
	}

	active = false;
	delegate->safeModFinished();
}

void MKSafeMod::threadFcn()
{
	Time sleepTime(0,THREAD_SLEEP_TIME_US);
	while(!threadStopRequest) {
		//ROS_INFO("Print!");
		// Check Timer
		bool emergency = false;
		if (emergencyLandingRequest) {
			ROS_ERROR("Emergency Landing Request received!");
			emergency = true;
		}

		if (cmdTimer.getElapsed() > Time(0,options.tCmdTimeoutUs->getValue())) {
			ROS_ERROR("Emergency! command Timeout too big! Elapsed: %f", cmdTimer.getElapsed().toDSec());
			emergency = true;
		}

		if (emergency) {
			ROS_ERROR("Emergency Situation found!");
			if (emergencyLandingRequest) {
				ROS_ERROR("EmergencyRequest: true");
			} else {
				ROS_ERROR("EmergencyRequest: false");
			}
			ROS_ERROR_STREAM("Timer: " << cmdTimer.getElapsed().toString());

			emergencyLandingRequest = false;
			safeModFcn();
			//threadStopRequest = true;
		}

		// sleep
		sleepTime.sleep();
	}
}

void MKSafeMod::startThread()
{
	if (!mkSafeModThread) {
		threadStopRequest = false;
		mkSafeModThread = new boost::thread(&MKSafeMod::threadFcn, this);
	} else {
		ROS_WARN("Trying to start MKSafeModThread, but it's already active!");
	}

}

void MKSafeMod::stopThread()
{
	if (mkSafeModThread) {
		threadStopRequest = true;
		mkSafeModThread->join();
		delete mkSafeModThread;
		mkSafeModThread = NULL;
	} else {
		ROS_WARN("Trying to stop MKSafeModThread, but it does not exist!");
	}
}

void MKSafeMod::start()
{
	startThread();
}

void MKSafeMod::stop()
{
	stopThread();
}

// returns true if SafeMod has Taken Over
bool MKSafeMod::isActive() const
{
	return active;
}

bool MKSafeMod::isRunning() const
{
	return (mkSafeModThread != NULL);
}

void MKSafeMod::resetCmdTimer()
{
	cmdTimer.reset();
}

void MKSafeMod::setEmergency()
{
	ROS_INFO("Received Emergency Landing Request!");
	emergencyLandingRequest = true;
}


} /* namespace telekyb */
