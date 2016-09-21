/*
 * ROSBagBaseMsg.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: mriedel
 */

#include "ROSBagBaseMsg.hpp"

#include <iostream>

namespace TELEKYB_NAMESPACE {

namespace ROSBagMsgNS {

// private
// definition
std::map< std::string, ROSBagBaseMsg* > ROSBagBaseMsg::definedMsgs;

ROSBagBaseMsg::ROSBagBaseMsg() {
	std::cout << "Entering Constructor ROSBagBaseMsg" << std::endl;



//	definedMsgs.insert(std::make_pair("test", (ROSBagBaseMsg*)1));

	std::cout << "Leaving Constructor ROSBagBaseMsg" << std::endl;
}

ROSBagBaseMsg::~ROSBagBaseMsg() {
	// TODO Auto-generated destructor stub
}

bool ROSBagBaseMsg::hasMsg(const std::string& dataType)
{
	std::map< std::string, ROSBagBaseMsg* >::iterator it;
	it = definedMsgs.find(dataType);

	return (it != definedMsgs.end());
}

ROSBagBaseMsg* ROSBagBaseMsg::getROSBagMsg(const std::string& dataType)
{
	ROSBagBaseMsg* retValue = NULL;

	std::map< std::string, ROSBagBaseMsg* >::iterator it;
	it = definedMsgs.find(dataType);

	if (it != definedMsgs.end())
	{
		retValue = it->second;
	}

	return retValue;
}


}

} /* namespace telekyb */
