/*
 * MKInterfaceConnection.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKInterfaceConnection.hpp>

#include <telekyb_serial/SerialHelper.hpp>


// Boost Filesystem and Regex to Find Serial
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

// ADDR
#define ALLADDR 'a'
#define FCADDR 'b'
#define NCADDR 'c'
#define MK3MAGADDR 'd'

// Commands OUT
#define SETVALUE_CMD_OUT 's'
#define UPDATEVALUE_CMD_OUT 'u'
#define ONLY_CMD_OUT 'b'
#define ACTIVEDATAIDS_OUT 'l'

// Commands IN
#define VALUE_IN 'S'
#define ACTIVEDATA_IN 'D'
#define ACTIVEDATAIDS_IN 'L'


// Performance Parameters in Options?
#define SYNC_SLEEP_USEC 25000 // 25ms every 4th command at 120hz
#define MKVALUE_RESENDS 40

#define BUFFER_SIZE 256

#define SERIALDEVICE_MAXRETRIES 25

namespace TELEKYB_NAMESPACE
{

const char MKInterfaceConnection::setPrefix[] = {'#', FCADDR ,SETVALUE_CMD_OUT};
const char MKInterfaceConnection::updatePrefix[] = {'#', FCADDR ,UPDATEVALUE_CMD_OUT};
const char MKInterfaceConnection::cmdOnlyPrefix[] = {'#', FCADDR ,ONLY_CMD_OUT};
const char MKInterfaceConnection::setActiveDataIDsPrefix[] = {'#', FCADDR ,ACTIVEDATAIDS_OUT};


MKInterfaceConnection::MKInterfaceConnection(const std::string& devicePath)
	: ThreadedSerialDevice(devicePath)
{
	// Configure Connection
	setTermiosAttrCFlag(options.tTermiosCFlags->getValue());
	setTermiosAttrSpeed(options.tBaudRate->getValue().value(),options.tBaudRate->getValue().value());

	// Init activeDataIds
	for (int i = 0; i < ACTIVEDATA_SIZE; i++) {
		activeDataIDs.ids[i] = i; // 0,1,2,3,4,5,6,7,8,9
	}

	// set yourself as listener
	registerSerialDeviceListener(this);


	syncSetValue = NULL;
	syncUpdateValue = NULL;
	syncActiveDataIDsRequest = NULL;

	currentSendQueue = SENDQUEUE_ACTIVEDATAIDS;
}

MKInterfaceConnection::~MKInterfaceConnection()
{
	//device->unRegisterSerialDeviceListener(this);
	// This is done by child!!!
	//delete device;
}

void MKInterfaceConnection::handleReadSerialData(const std::vector<char>& data)
{
//	std::cout << "MKInterfaceConnection recv: ";
//	for (unsigned int i = 0; i < data.size(); i++)
//	{
//		std::cout << "--" << (int)data[i];
//	}
//	std::cout<< std::endl;

	// check CRC
	if (!SerialHelper::checkCRC(&data[0], data.size())) {
		ROS_WARN("Recv Package with wrong CRC!");
		return;
	} else {
		//ROS_INFO("CRC OK!");
	}

	// data[0] = '#' Otherwise unknown Message
	// data[1] = ADDRFIELD
	// data[2] = COMMANDFIELD

	if (data[0] != '#') {
		ROS_ERROR("Received an unknown Message from the MK.");
		return;
	}

	// big switch
	switch (data[1]) { // Adress Field
		case FCADDR: { // Flight Control
			switch (data[2]) { // Command Field
				case ACTIVEDATA_IN: {
					// do not write data if there is currently a request to change active DataIDs;
					if (syncActiveDataIDsRequest) {
						break;
					}

					// update Values
					MKActiveValues activeDataValues;
					SerialHelper::decodeData((char*)&activeDataValues, sizeof(activeDataValues), &data[3]);

					// write data to Values:
					for (int i = 0; i < ACTIVEDATA_SIZE; i++) {
						MKSingleValuePacket packet(activeDataIDs.ids[i], activeDataValues.values[i]);
						handleRecvSingleValuePacket(packet);
					}

					break;
				}
				case ACTIVEDATAIDS_IN: {
					MKActiveIDs newActiveDataIDs;
					SerialHelper::decodeData((char*)&newActiveDataIDs, sizeof(newActiveDataIDs), &data[3]);
					// immediately set
					activeDataIDs = newActiveDataIDs;

					// notify sync and async

					// SYNC
					if (syncActiveDataIDsRequest && *syncActiveDataIDsRequest == activeDataIDs) {
						// OK SYNC successful
						syncActiveDataIDsRequest = NULL;
					}

					// ASYNC
					if (activeDataIDs == activeDataIDsQueue.front()) {
						// take it off
						boost::mutex::scoped_lock queueLock(queueMutex);
						activeDataIDsQueue.pop();
					}


					break;
				}
				case VALUE_IN: {
					MKSingleValuePacket singleValuePacket(0,0);
					SerialHelper::decodeData((char*)&singleValuePacket, sizeof(singleValuePacket), &data[3]);
					handleRecvSingleValuePacket(singleValuePacket);
					//ROS_INFO("ID: %d, Value: %d", singleValuePacket.id, singleValuePacket.value);
					break;
				}
				default:
					ROS_ERROR("FC Received an Message with unknown Command Field %c", data[2]);
					break;

			}
			break;
		}
		default:
			ROS_ERROR("Received an Message with unknown Adress Field %c", data[1]);
			break;
	}

}

// Receive Handler
void MKInterfaceConnection::handleRecvSingleValuePacket(MKSingleValuePacket packet)
{
	// ALWAYS SET! // Timestamp is updated here!
	MKValue* newValue = mkData.setValue(packet);

	if (!newValue) {
		ROS_ERROR("mkData.setValue(packet) returned NULL-Pointer. This should never happen! MK Code wrong!?!");
		return;
	}

	// SYNC Methods
	// Update
	if (syncUpdateValue && *syncUpdateValue == packet.id) {
		// Time is implicit
		syncUpdateValue = NULL;
	}

	// Set
	if (syncSetValue && *syncSetValue == packet) {
		// update Time
		// Set successful. Set NULL.
		syncSetValue = NULL;
	}


	// ASYNC Handlers

	// if on top of queue. take it off
	if (!updateQueue.empty() && packet.id == updateQueue.front()) {
		boost::mutex::scoped_lock queueLock(queueMutex);
		updateQueue.pop();
	}

	if (!setQueue.empty() && packet == setQueue.front()) {
		boost::mutex::scoped_lock queueLock(queueMutex);
		setQueue.pop();
	}
}


// write // expects new Command! // stores lastCmd // checks queues // does not check Pointer
void MKInterfaceConnection::writeCommand(const MKCommandsPacket& command)
{
	char buffer[BUFFER_SIZE];
	char msg[BUFFER_SIZE];
	int nBytes = 0;

	boost::mutex::scoped_lock queueLock(queueMutex);

	// TODO: Select next Queue.
	for (int i = 0; i < SENDQUEUE_SIZE; ++i) {
		if (currentSendQueue == SENDQUEUE_ACTIVEDATAIDS && !activeDataIDsQueue.empty()) {
			break;
		} else if (currentSendQueue == SENDQUEUE_SET && !setQueue.empty()) {
			break;
		} else if (currentSendQueue == SENDQUEUE_UPDATE && !updateQueue.empty()) {
			break;
		}

		currentSendQueue = (SendQueue)((currentSendQueue+1) % SENDQUEUE_SIZE);
	}



	// TODO: Implement MAX_TRIES? try sending 3 times then pop? Will send indefinitely now!
	if (currentSendQueue == SENDQUEUE_ACTIVEDATAIDS && !activeDataIDsQueue.empty()
			&& asyncActiveIDsTimer.frequency() < options.tAsyncSendFrequency->getValue()) {
		asyncActiveIDsTimer.reset();

		MKActiveIDs activeDataIDsRequest = activeDataIDsQueue.front();
		queueLock.unlock();

		memcpy(msg,&command, sizeof(command));
		memcpy(msg + sizeof(command), &activeDataIDsRequest, sizeof(activeDataIDsRequest));

		nBytes = SerialHelper::encodeData(buffer, setActiveDataIDsPrefix, sizeof(updatePrefix),
				msg, sizeof(command) + sizeof(activeDataIDsRequest));

	} else if (currentSendQueue == SENDQUEUE_SET && !setQueue.empty()
			&& asyncSetTimer.frequency() < options.tAsyncSendFrequency->getValue()) {
		asyncSetTimer.reset();

		MKSingleValuePacket setValueRequest = setQueue.front();
		queueLock.unlock();

		// build message
		memcpy(msg,&command, sizeof(command));
		memcpy(msg + sizeof(command), &setValueRequest, sizeof(setValueRequest));

		nBytes = SerialHelper::encodeData(buffer, setPrefix, sizeof(setPrefix),
				msg, sizeof(command) + sizeof(setValueRequest));

	} else if (currentSendQueue == SENDQUEUE_UPDATE && !updateQueue.empty()
			&& asyncUpdateTimer.frequency() < options.tAsyncSendFrequency->getValue()) {
		asyncUpdateTimer.reset();
		MKInt updateValueRequest = updateQueue.front();
		queueLock.unlock();

		ROS_INFO("Sending updateValue with ID: %d", updateValueRequest);

		memcpy(msg,&command, sizeof(command));
		memcpy(msg + sizeof(command), &updateValueRequest, sizeof(updateValueRequest));

		nBytes = SerialHelper::encodeData(buffer, updatePrefix, sizeof(updatePrefix),
				msg, sizeof(command) + sizeof(updateValueRequest));

	} else {
		nBytes = SerialHelper::encodeData(buffer, cmdOnlyPrefix, sizeof(cmdOnlyPrefix),
				(const char*)&command, sizeof(command));
	}

	// put on Serial
	try {
		writeDevice(buffer,nBytes);
	} catch (SerialException &e) {
		e.process();
	}

	currentSendQueue = (SendQueue)((currentSendQueue+1) % SENDQUEUE_SIZE);
}
void MKInterfaceConnection::writeSetValue()
{
	if (!syncSetValue) {
		// NULL -> should not occur since it's only called from setValue
		return;
	}

	MKSingleValuePacket packet = *syncSetValue;

	// build message
	char msg[BUFFER_SIZE];
	memcpy(msg,&lastCmd, sizeof(lastCmd));
	memcpy(msg + sizeof(lastCmd), &packet, sizeof(packet));

	char buffer[BUFFER_SIZE];
	int nBytes = SerialHelper::encodeData(buffer, setPrefix, sizeof(setPrefix), msg, sizeof(lastCmd) + sizeof(packet));

	// put on Serial
	try {
		writeDevice(buffer,nBytes);
	} catch (SerialException &e) {
		e.process();
	}
}

void MKInterfaceConnection::writeUpdateValue()
{
	// copy, because otherwiese the receiving end could possible put it to NULL at any point!
	if (!syncUpdateValue) {
		// NULL -> should not occur since it's only called from updateValue
		return;
	}

	MKInt valueID = *syncUpdateValue;

	// build message
	char msg[BUFFER_SIZE];
	memcpy(msg,&lastCmd, sizeof(lastCmd));
	memcpy(msg + sizeof(lastCmd), &valueID, sizeof(valueID));

	char buffer[BUFFER_SIZE];
	int nBytes = SerialHelper::encodeData(buffer, updatePrefix, sizeof(updatePrefix), msg, sizeof(lastCmd) + sizeof(valueID));

	// put on Serial
	try {
		writeDevice(buffer,nBytes);
	} catch (SerialException &e) {
		e.process();
	}
}

void MKInterfaceConnection::writeActiveDataIDsRequest()
{
	if (!syncActiveDataIDsRequest) {
		// NULL
		return;
	}

	MKActiveIDs activeDataIDsRequestCopy = *syncActiveDataIDsRequest;
	// build message
	char msg[BUFFER_SIZE];
	memcpy(msg,&lastCmd, sizeof(lastCmd));
	memcpy(msg + sizeof(lastCmd), &activeDataIDsRequestCopy, sizeof(activeDataIDsRequestCopy));

	char buffer[BUFFER_SIZE];
	int nBytes = SerialHelper::encodeData(buffer, setActiveDataIDsPrefix, sizeof(updatePrefix), msg, sizeof(lastCmd) + sizeof(activeDataIDsRequestCopy));

	// put on Serial
	try {
		writeDevice(buffer,nBytes);
	} catch (SerialException &e) {
		e.process();
	}
}

// setLastCommand <- Mutex protected!
void MKInterfaceConnection::setLastCmd(const MKCommandsPacket& command)
{
	boost::mutex::scoped_lock(lastCmdMutex);
	lastCmd = command;
}

// send command
void MKInterfaceConnection::sendCommand(const MKCommandsPacket& command)
{
	// write on Serial / Check Queues
	writeCommand(command);
	// only set after sending! -> Data must reach UAV asap
	setLastCmd(command);
}

// set value <- reference to the acutal value!
bool MKInterfaceConnection::setValue(MKSingleValuePacket value)
{
	// must be member of MKData
	MKValue* mkValue = mkData.getValueByID(value.id);
	if (!mkValue) {
		ROS_ERROR("Called (sync) setValue with unknown MKValue ID.");
		return false;
	}

	boost::mutex::scoped_lock(syncSetValueMutex);
	syncSetValue = &value;


	Time sleepTime(0,SYNC_SLEEP_USEC);

	for (int i = 0; i < MKVALUE_RESENDS; ++i) {

		// sent
		writeSetValue();

		// sleep
		sleepTime.sleep();

		// check <- syncSetValue get's set to NULL if success!!!
		if (syncSetValue == NULL) {
			return true;
		}
	}

	// not successful!
	syncSetValue = NULL;
	return false;
}

// update value <- get value write result in reference
bool MKInterfaceConnection::updateValue(MKInt id)
{
	// must be member of MKData
	MKValue* mkValue = mkData.getValueByID(id);
	if (!mkValue) {
		ROS_ERROR("Called (sync) updateValue with unknown MKValue ID.");
		return false;
	}

	boost::mutex::scoped_lock(syncUpdateValueMutex);
	syncUpdateValue = &id;

	// 10ms sleep
	Time sleepTime(0,SYNC_SLEEP_USEC);

	for (int i = 0; i < MKVALUE_RESENDS; ++i) {

		// sent
		writeUpdateValue();

		// sleep
		sleepTime.sleep();

		// check <- syncUpdateValue get's set to NULL if success!!!
		if (syncUpdateValue == NULL) {
			return true;
		}
	}

	// not successful
	syncUpdateValue = NULL;

	return false;
}

bool MKInterfaceConnection::setActiveDataIDs(MKActiveIDs activeDataIDs_)
{
	// check if Values are ok.
	if (! isValidMKActiveIDs(activeDataIDs_)) {
		ROS_ERROR("MKInterfaceConnection::setActiveDataIDs with inValid MKActiveIDs Object");
		return false;
	}

	boost::mutex::scoped_lock(syncActiveDataIDRequestMutex);
	syncActiveDataIDsRequest = &activeDataIDs_;

	// 10ms sleep Option?
	Time sleepTime(0,SYNC_SLEEP_USEC);

	for (int i = 0; i < MKVALUE_RESENDS; ++i) {

		// sent
		writeActiveDataIDsRequest();

		// sleep
		sleepTime.sleep();

		// check <- syncUpdateValue get's set to NULL if success!!!
		if (syncActiveDataIDsRequest == NULL) {
			return true;
		}
	}

	// not successful
	syncActiveDataIDsRequest = NULL;

	return false;
}

// set value <- reference to the acutal value! // Store Value in List
void MKInterfaceConnection::setValueAsync(MKSingleValuePacket value)
{
	// Queue Limit
	if (setQueue.size() > SET_QUEUE_SIZE) {
		ROS_ERROR("Set Queue is full! Limit: %d", SET_QUEUE_SIZE);
		return;
	}

	// must be member of MKData
	MKValue* mkValue = mkData.getValueByID(value.id);
	if (!mkValue) {
		ROS_ERROR("Called setValueAsync with unknown MKValue ID.");
		return;
	}

	// add to queue
	boost::mutex::scoped_lock queueLock(queueMutex);
	setQueue.push(value);
	ROS_INFO("Added %d ID to SetQueue", value.id);
}

// update value <- get value write result in reference
void MKInterfaceConnection::updateValueAsync(MKInt id) // Store Value in List
{
	// Queue Limit
	if (updateQueue.size() > UPDATE_QUEUE_SIZE) {
		ROS_ERROR("Update Queue is full! Limit: %d", UPDATE_QUEUE_SIZE);
		return;
	}

	// must be member of MKData
	MKValue* mkValue = mkData.getValueByID(id);
	if (!mkValue) {
		ROS_ERROR("Called updateValueAsync with unknown MKValue ID.");
		return;
	}

	// add to queue
	boost::mutex::scoped_lock queueLock(queueMutex);
	updateQueue.push(id);
	ROS_INFO("Added %d ID to UpdateQueue", id);
}

void MKInterfaceConnection::setActiveDataIDsAsync(MKActiveIDs activeDataIDs_)
{
	// Queue Limit
	if (activeDataIDsQueue.size() > ACTIVE_IDS_QUEUE_SIZE) {
		ROS_ERROR("Active Data IDs Queue is full! Limit: %d", ACTIVE_IDS_QUEUE_SIZE);
		return;
	}

	// check if Values are ok.
	if (! isValidMKActiveIDs(activeDataIDs_)) {
		ROS_ERROR("MKInterfaceConnection::setActiveDataIDsAsync with inValid MKActiveIDs Object");
		return;
	}

	boost::mutex::scoped_lock queueLock(queueMutex);
	activeDataIDsQueue.push(activeDataIDs_);
}

MKActiveIDs MKInterfaceConnection::getActiveDataIDs() const
{
	return activeDataIDs;
}

const MKData& MKInterfaceConnection::getMKDataRef() const
{
	return mkData;
}

void MKInterfaceConnection::registerMKDataListener(MKDataListener* listener)
{
	mkData.registerMKDataListener(listener);
}

void MKInterfaceConnection::unRegisterMKDataListener(MKDataListener* listener)
{
	mkData.unRegisterMKDataListener(listener);
}


// Find Connetion that meets conditions
MKInterfaceConnection* MKInterfaceConnection::findConnection(
		const std::string& serialDeviceDirectory,
		const std::string& serialDeviceNameRegEx,
		std::vector<MKSingleValuePacket> conditions)
{
	MKInterfaceConnection* connection = NULL;

	if (!fs::is_directory(serialDeviceDirectory)) {
		ROS_ERROR_STREAM(serialDeviceDirectory << "does not exist.");
		return connection;
	}

	boost::regex filter( serialDeviceNameRegEx );

	fs::directory_iterator end_itr;
	// list of matching paths
	std::vector<fs::path> serialPaths;
	for (fs::directory_iterator it( serialDeviceDirectory ); it != end_itr; ++it)
	{
	  if ( ! boost::regex_match(it->path().filename().string(), filter)) {
			// no match
			continue;
		}

	  // put path in list
	  serialPaths.push_back(it->path());
	}

	// retry vector. -1 means no more retries needed
	// > 0 number of retries so far.
	// stop if all values are > options.tDeviceMaxRetries
	std::vector<int> serialRetries(serialPaths.size(), 0);

	bool serialBreakCondition = false;
	// Check all Paths in List!
	while (connection == NULL && !serialBreakCondition)
	{
		// Break if no serial is valid any more
		serialBreakCondition = true;
		for (unsigned int path = 0; path < serialPaths.size(); ++path) {

			// skip serial if -1 or > options.tDeviceMaxRetries->getValue()
			if (serialRetries[path] == -1 || serialRetries[path] > SERIALDEVICE_MAXRETRIES) {
				continue;
			}
			serialBreakCondition = false;

			//ROS_INFO("Checking Connection: %s!", (*path_it).string().c_str() );
		  	try {
				connection = new MKInterfaceConnection(serialPaths[path].string());
			} catch (SerialException &e) {
				if (e.code != SerialExceptionCode::LOCKED) {
					e.process();
					// do not check any more
					serialRetries[path] = -1;
				} else {
					// locked
					//e.process();
					if (serialRetries[path] >= SERIALDEVICE_MAXRETRIES) {
						ROS_ERROR("Unable to access serial Device after %d tries.", serialRetries[path]);
					}
					serialRetries[path]++;
				}
				continue;
			}


			for (unsigned int i = 0; i < conditions.size(); ++i)
			{
				// get right MKValue Pointer
				// Copy
				MKValue* valuePtr = connection->getMKDataRef().getValueByID(conditions[i].id);
				// NOTE: UpdateValue checks for NULL Pointer. So we don't have to do it.

				if (!connection->updateValue(conditions[i].id)) {
					ROS_ERROR("Unable to update Value from QC");
					delete connection; connection = NULL;
					break;
				}

				if ( ! valuePtr->equals(conditions[i]) ) {
					//ROS_INFO("Error in getting or comparing Value!");
					// This is not the Connection
					ROS_ERROR_STREAM("Mismatch condition! Name: "
							<< MKDataDefines::MKDATAIDS_NAMES[conditions[i].id]
					        << " (" << conditions[i].id << ") Is: " << valuePtr->getValue() << " Should be: " << conditions[i].value);
					delete connection; connection = NULL;
					break;
				}
			}

			// if we still have a connection here. We found our match
			if ( connection ) {
				// found
				ROS_INFO_STREAM("Found matching MKInterfaceConnection on serial: " << serialPaths[path].string());
				break;
			} else {
				// don't test this connection anymore
				serialRetries[path] = -1;
			}

		}

		// sleep a little
		if (! connection) {
			usleep(50 * 1000);
		}
	}

	if (!connection) {
		// Could not find connection
		ROS_ERROR("Unable to find connection that matches conditions:");
		for (unsigned int i = 0; i < conditions.size(); ++i)
		{
			ROS_ERROR_STREAM("Name: "
					<< MKDataDefines::MKDATAIDS_NAMES[conditions[i].id]
			        << " (" << conditions[i].id << ") Value: " << conditions[i].value);
		}
	}

	return connection;
}

}

