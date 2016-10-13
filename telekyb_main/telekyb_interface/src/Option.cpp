/*
 * Option.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/Option.hpp>

namespace TELEKYB_INTERFACE_NAMESPACE {

//Option::Option()
//: optionNSNodehandle(ros::NodeHandle()),
//  optionName(std::string())
//{
//
//}

Option::Option(const ros::NodeHandle& optionNSNodehandle_, const std::string& optionName_)
	: optionNSNodehandle(optionNSNodehandle_),
	  optionName(optionName_)
{

}

Option::~Option()
{

}

bool Option::exists()
{
	return optionNSNodehandle.hasParam(optionName);
}


} /* namespace telekyb_interface */
