/*
 * MKValue.cpp
 *
 *  Created on: Nov 25, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKValue.hpp>

// lexical cast
#include <boost/lexical_cast.hpp>

namespace TELEKYB_NAMESPACE {

MKValue::MKValue(const std::string& name_, MKInt id_)
	: name(name_), id(id_), value(0), stamp(Time::Zero())
{
	// TODO Auto-generated constructor stub

}

MKValue::~MKValue()
{
	// TODO Auto-generated destructor stub
}

bool MKValue::isUnset() const
{
	return stamp.isZero();
}

MKSingleValuePacket MKValue::getMKSingleValuePacket() const
{
	return MKSingleValuePacket(id,value);
}

MKSingleValuePacket MKValue::getMKSingleValuePacketWithValue(MKInt value_) const
{
	return MKSingleValuePacket(id, value_);
}

//MKSingleIdPacket MKValue::getMKSingleIdPacket() const
//{
//	MKSingleIdPacket packet;
//	packet.id = id;
//	return packet;
//}

// Setters / Getters
MKInt MKValue::getID() const
{
	return id;
}
//void MKValue::setID(MKInt id_)
//{
//	id = id_;
//}

MKInt MKValue::getValue() const
{
	return value;
}
void MKValue::setValue(MKInt value_)
{
	//ROS_INFO_STREAM("Updated " << name << " with " << value_);
	stamp = Time(); // current Time
	value = value_;
}

std::string MKValue::toString() const
{
	return name + "(" + boost::lexical_cast<std::string>(id) + ") " + boost::lexical_cast<std::string>(value);
}


} /* namespace telekyb */
