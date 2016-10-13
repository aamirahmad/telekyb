/*
 * MKData.hpp
 *
 *  Created on: Nov 25, 2011
 *      Author: mriedel
 */

#ifndef MKDATA_HPP_
#define MKDATA_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_mkinterface/MKValue.hpp>
#include <map>

#include <set>

namespace TELEKYB_NAMESPACE {

typedef std::map<MKInt,MKValue*> MKValueMap;

// MKDataListener
class MKDataListener {
public:
	virtual ~MKDataListener() {};

	// this is only called on updates
	virtual void dataValueUpdated(MKValue* value) = 0;
	// this is always called (set/update)
	//virtual void dataValueUpdated(MKValue* value) = 0;
};

class MKData {
protected:
	MKValueMap valueMap;
	// Value Definition goes in here.
	void createMap();

	std::set<MKDataListener*> dataListenerSet;

	// only by MKInterfaceConnection
	MKValue* setValue(MKSingleValuePacket packet);

	void notifyMKDataListeners(MKValue* value);

public:
	MKData();
	virtual ~MKData();

	MKValue* getValueByID(MKInt id) const;
	bool isMember(MKValue* value_) const;

	void registerMKDataListener(MKDataListener* listener);
	void unRegisterMKDataListener(MKDataListener* listener);


	// Some collections of MKActiveIDs
	static MKActiveIDs getPattern(MKDataPattern pattern);
	// static is Accessor

	// friend class
	friend class MKInterfaceConnection;

};

} /* namespace telekyb */
#endif /* MKDATA_HPP_ */
