/*
 * MKInterfaceConnection.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef MKINTERFACECONNECTION_HPP_
#define MKINTERFACECONNECTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_serial/ThreadedSerialDevice.hpp>

#include <telekyb_defines/MKDefines.hpp>
#include <tk_mkinterface_outdoor/MKData.hpp>
#include <tk_mkinterface_outdoor/MKValue.hpp>

// Boost locking
#include <boost/thread/mutex.hpp>

// Options
#include <tk_mkinterface_outdoor/MKInterfaceConnectionOptions.hpp>


// Queue
#include <queue>

// Visible Defines.  (for ROSINTERFACE)
#define SET_QUEUE_SIZE 50
#define UPDATE_QUEUE_SIZE 50
#define ACTIVE_IDS_QUEUE_SIZE 10

namespace TELEKYB_NAMESPACE
{

// TODO: This Element is not copyable. (Because it's a Listener!!!)

class MKInterfaceConnection : public ThreadedSerialDevice, public SerialDeviceListener {
protected:
	// The prefixpart of the message to UAV
	static const char setPrefix[];
	static const char updatePrefix[];
	static const char cmdOnlyPrefix[];
	static const char setActiveDataIDsPrefix[];

	//Options
	MKInterfaceConnectionOptions options;

	// The Data Structure for that connection
	MKData mkData;
	//MKInt activeDataIDs[ACTIVEDATA_SIZE];
	MKActiveIDs activeDataIDs;

	// Last Command 0 if not yet set. // This is important if other commands should send instantly
	boost::mutex lastCmdMutex;
	MKCommandsPacket lastCmd;

	// ActiveData IDs
	std::queue<MKActiveIDs> activeDataIDsQueue; // ASYNC
	MKActiveIDs* syncActiveDataIDsRequest; // SYNC -> != NULL for request
	boost::mutex syncActiveDataIDRequestMutex; // SYNC

	// Sync SET/UPDATE
	boost::mutex syncSetValueMutex; // SYNC
	boost::mutex syncUpdateValueMutex; //SYNC

	// these are != NULL if a sync Request is present
	MKInt* syncUpdateValue; // SYNC
	MKSingleValuePacket* syncSetValue; // SYNC

	// set and update Queues ASYNC
	std::queue<MKInt> updateQueue;
	std::queue<MKSingleValuePacket> setQueue;

	// Timer to determine package spacing.
	Timer asyncUpdateTimer;
	Timer asyncSetTimer;
	Timer asyncActiveIDsTimer;

	// specify the current sendQueue
	SendQueue currentSendQueue;

	// Queue Mutex <- so that we do not pop when calling front().
	boost::mutex queueMutex;


	// write // expects new Command! // stores lastCmd // checks queues // does not check Pointer
	void writeCommand(const MKCommandsPacket& command);

	// write Values & (old command). Does not check Queue / Checks pointers (Sync)
	void writeSetValue(); // SYNC
	void writeUpdateValue(); // SYNC
	void writeActiveDataIDsRequest(); // SYNC

	// setLastCommand <- Mutex protected!
	void setLastCmd(const MKCommandsPacket& command);


	// Handle Received Data
	void handleRecvSingleValuePacket(MKSingleValuePacket packet);



public:
	// BEWARE. This throws SerialExceptions
	MKInterfaceConnection(const std::string& devicePath);
	virtual ~MKInterfaceConnection();

	// handle Data
	void handleReadSerialData(const std::vector<char>& data);

	// send command
	void sendCommand(const MKCommandsPacket& command);

	// set value <- reference to the acutal value! Setting stamp. so cannot be const!
	// Never use these for automatic progam logic!
	bool setValue(MKSingleValuePacket value);

	// update value <- get value write result in reference
	// Never use these for automatic program logic!
	bool updateValue(MKInt id);

	// sync Method to set active Data IDs;
	bool setActiveDataIDs(MKActiveIDs activeDataIDs_);

	// set value <- reference to the acutal value! // Store Value in List // Setting stamp. so cannot be const!
	void setValueAsync(MKSingleValuePacket value);

	// update value <- get value write result in reference
	void updateValueAsync(MKInt id); // Store Value in List

	// activeDataIDs can only be set async! // careful about the requirements!
	void setActiveDataIDsAsync(MKActiveIDs activeDataIDs_);

	// get the current MKActiveIDs
	MKActiveIDs getActiveDataIDs() const;

	// Data Accessors
	const MKData& getMKDataRef() const;

	// Listeners
	void registerMKDataListener(MKDataListener* listener);
	void unRegisterMKDataListener(MKDataListener* listener);

	// inputs
	static MKInterfaceConnection* findConnection(
			const std::string& serialDeviceDirectory,
			const std::string& serialDeviceNameRegEx,
			std::vector<MKSingleValuePacket> conditions);

};

} // namespace

#endif /* MKINTERFACECONNECTION_HPP_ */
