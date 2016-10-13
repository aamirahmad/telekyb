/*
 * MKData.cpp
 *
 *  Created on: Nov 25, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKData.hpp>

#include <boost/foreach.hpp>

namespace TELEKYB_NAMESPACE {
MKData::MKData()
{
	// create Map
	createMap();
}

MKData::~MKData()
{
	MKValueMap::iterator it;
	for (it = valueMap.begin(); it != valueMap.end(); it++) {
		delete (*it).second;
	}
}

void MKData::createMap()
{
	for (int i = 0; i < MKDataDefines::MKDATAIDS_NUM; ++i) {
		valueMap[i] = new MKValue( MKDataDefines::MKDATAIDS_NAMES[i],i);
	}
}

MKValue* MKData::getValueByID(MKInt id) const
{
	MKValue* value = NULL;
	if (valueMap.count(id) > 0) {
		value = valueMap.at(id);
	}
	return value;
}

MKValue* MKData::setValue(MKSingleValuePacket packet)
{
	MKValue* value = getValueByID(packet.id);
	if (value) {
		value->setValue(packet.value);
		// notify
		notifyMKDataListeners(value);
	}
	// NULL means not successful!
	return value;
}

void MKData::notifyMKDataListeners(MKValue* value)
{
	BOOST_FOREACH(MKDataListener* l, dataListenerSet) {
		l->dataValueUpdated(value);
	}
}

bool MKData::isMember(MKValue* value_) const
{
	return value_ && (value_ == getValueByID(value_->getID()));
}

MKActiveIDs MKData::getPattern(MKDataPattern pattern)
{
	MKActiveIDs activeIDs;
	switch (pattern.value()) {
		case MKDataPattern::RawImuAttEst:
			activeIDs.ids[0] = MKDataDefines::RAW_ACC_X;
			activeIDs.ids[1] = MKDataDefines::RAW_ACC_Y;
			activeIDs.ids[2] = MKDataDefines::RAW_ACC_Z;
			activeIDs.ids[3] = MKDataDefines::RAW_GYRO_X;
			activeIDs.ids[4] = MKDataDefines::RAW_GYRO_Y;
			activeIDs.ids[5] = MKDataDefines::RAW_GYRO_Z;
			activeIDs.ids[6] = MKDataDefines::PITCH;
			activeIDs.ids[7] = MKDataDefines::ROLL;
			activeIDs.ids[8] = MKDataDefines::MOTOR_STATE;
			activeIDs.ids[9] = MKDataDefines::TIME_MS_MOD_60S;
			break;
		case MKDataPattern::AccOffsetGyroDrift:
			activeIDs.ids[0] = MKDataDefines::DRIFT_GYRO_X;
			activeIDs.ids[1] = MKDataDefines::DRIFT_GYRO_Y;
			activeIDs.ids[2] = MKDataDefines::DRIFT_GYRO_Z;
			activeIDs.ids[3] = MKDataDefines::OFFSET_RAW_ACC_X;
			activeIDs.ids[4] = MKDataDefines::OFFSET_RAW_ACC_Y;
			activeIDs.ids[5] = MKDataDefines::OFFSET_RAW_ACC_Z;
			activeIDs.ids[6] = MKDataDefines::DRIFT_ESTIM_ACTIVE;
			activeIDs.ids[7] = MKDataDefines::MIRROR_DATA_ACTIVE;
			activeIDs.ids[8] = MKDataDefines::MOTOR_STATE;
			activeIDs.ids[9] = MKDataDefines::TIME_MS_MOD_60S;
			break;
		case MKDataPattern::CompFilt:
			activeIDs.ids[0] = MKDataDefines::PITCH;
			activeIDs.ids[1] = MKDataDefines::ROLL;
			activeIDs.ids[2] = MKDataDefines::PITCH_RATE;
			activeIDs.ids[3] = MKDataDefines::ROLL_RATE;
			activeIDs.ids[4] = MKDataDefines::RAW_ACC_X;
			activeIDs.ids[5] = MKDataDefines::RAW_ACC_Y;
			activeIDs.ids[6] = MKDataDefines::RAW_ACC_Z;
			activeIDs.ids[7] = MKDataDefines::MIRROR_TIME_PERIOD;
			activeIDs.ids[8] = MKDataDefines::MOTOR_STATE;
			activeIDs.ids[9] = MKDataDefines::TIME_MS_MOD_60S;
			break;
		case MKDataPattern::Stats:
			activeIDs.ids[0] = MKDataDefines::RCVD_CMD_PER_SEC;
			activeIDs.ids[1] = MKDataDefines::MAX_RCVD_PERIOD_MS;
			activeIDs.ids[2] = MKDataDefines::SENT_PKT_PER_SEC;
			activeIDs.ids[3] = MKDataDefines::MAX_SENT_PERIOD_MS;
			activeIDs.ids[4] = MKDataDefines::CTRL_STEPS_PER_SEC;
			activeIDs.ids[5] = MKDataDefines::MAX_FLIGHT_CTRL_PERIOD_MS;
			activeIDs.ids[6] = MKDataDefines::BATT_VOLT;
			activeIDs.ids[7] = MKDataDefines::MIRROR_TIME_PERIOD;
			activeIDs.ids[8] = MKDataDefines::MOTOR_STATE;
			activeIDs.ids[9] = MKDataDefines::TIME_MS_MOD_60S;
			break;
		default:

			// Default Config
			for (int i = 0; i < ACTIVEDATA_SIZE; i++) {
				activeIDs.ids[i] = i;
			}


			break;
	}

	return activeIDs;
}

void MKData::registerMKDataListener(MKDataListener* listener)
{
	dataListenerSet.insert(listener);
}

void MKData::unRegisterMKDataListener(MKDataListener* listener)
{
	dataListenerSet.erase(listener);
}


} /* namespace telekyb */
