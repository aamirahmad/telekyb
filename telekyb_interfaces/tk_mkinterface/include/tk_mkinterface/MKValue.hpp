/*
 * MKValue.hpp
 *
 *  Created on: Nov 25, 2011
 *      Author: mriedel
 */

#ifndef MKVALUE_HPP_
#define MKVALUE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_defines/MKDefines.hpp>

#include <telekyb_base/Time.hpp>

#include <ros/console.h>

/**
 * Represents a Value on the UAV
 * Values are MKint / MKint Pairs
 */

namespace TELEKYB_NAMESPACE {

class MKValue {
protected:
	std::string name;
	MKInt id;
	MKInt value;
	Time stamp;

	// Only MKData can create Values
	MKValue(const std::string& name_, MKInt id_);

	// Set Value // Only allowed by MKData
	void setValue(MKInt value_);

public:
	virtual ~MKValue();

	bool isUnset() const;

	MKSingleValuePacket getMKSingleValuePacket() const;
	// For Setting
	MKSingleValuePacket getMKSingleValuePacketWithValue(MKInt value_) const;
	//MKSingleIdPacket getMKSingleIdPacket() const;

	// Setters / Getters
	MKInt getID() const;
	bool hasID(MKInt id_) const;
	//void setID(MKInt id_);

	MKInt getValue() const;

	// setStamp sets Stamp to current Time.
	void setStamp();
	Time getStamp() const;

	// getName
	std::string getName() const;

	// ID match?
//	bool equals(MKSingleIdPacket packet) const;

	// ID & Value match?
	bool equals(MKSingleValuePacket packet) const;

	// only check ID & Value!
	bool equals(const MKValue& other) const;

	std::string toString() const;

	// friend classes
	friend class MKData;

};

// Inline
// ID?
inline
bool MKValue::hasID(MKInt id_) const
{
	return id == id_;
}

// ID match?
//inline
//bool MKValue::equals(MKSingleIdPacket packet) const
//{
//	return packet.id == id;
//}

// ID & Value match?
inline
bool MKValue::equals(MKSingleValuePacket packet) const
{
	return (packet.id == id) && (packet.value == value);
}

inline
// only check ID & Value!
bool MKValue::equals(const MKValue& other) const
{
	return (other.id == id) && (other.value == value);
}

inline
void MKValue::setStamp()
{
	stamp = Time();
}

inline
Time MKValue::getStamp() const
{
	return stamp;
}

inline
std::string MKValue::getName() const
{
	return name;
}

} /* namespace telekyb */
#endif /* MKVALUE_HPP_ */
