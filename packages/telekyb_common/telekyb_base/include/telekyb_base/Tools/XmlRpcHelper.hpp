/*
 * XmlRpcHelper.h
 *
 *  Created on: Oct 12, 2011
 *      Author: mriedel
 */

#ifndef XMLRPCHELPER_HPP_
#define XMLRPCHELPER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <XmlRpc.h>

namespace TELEKYB_NAMESPACE
{

class XmlRpcHelper {
public:
	static bool xmlRpcValuetoString(XmlRpc::XmlRpcValue xmlValue, std::string& value);
};

} // namespace

#endif /* XMLRPCHELPER_H_ */
