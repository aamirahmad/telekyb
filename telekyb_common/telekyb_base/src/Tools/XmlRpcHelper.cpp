/*
 * XmlRpcHelper.cpp
 *
 *  Created on: Oct 12, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Tools/XmlRpcHelper.hpp>

#include <iostream>

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

namespace TELEKYB_NAMESPACE
{


// Multiple Level Arrays are not supported!
bool XmlRpcHelper::xmlRpcValuetoString(XmlRpc::XmlRpcValue xmlValue, std::string& value) {

	bool ret = false;

	switch (xmlValue.getType()) {
		case XmlRpc::XmlRpcValue::TypeArray: {
			// no multiple level support // only Number support (and boolean)
			std::stringstream ss;
			std::string temp;
			std::string seperator("");
			ret = true;
			for (int i = 0; i < xmlValue.size(); i++) {
				if (xmlValue[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean
						|| xmlValue[i].getType() == XmlRpc::XmlRpcValue::TypeDouble
						|| xmlValue[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
					xmlRpcValuetoString(xmlValue[i], temp);
					ss << seperator << temp;
					seperator = " ";
				}
				else {
					ret = false;
					break;
				}
			}
			// ok
			if (ret) {
				value = ss.str();
			}
			break;
		}
		case XmlRpc::XmlRpcValue::TypeBase64:
			// NOT IMPLEMENTED
			break;
		case XmlRpc::XmlRpcValue::TypeBoolean:
			if ((bool)xmlValue) {
				value = "true";
			} else {
				value = "false";
			}
			//std::cout << "BoolValue: " << value << std::endl;
			//value = lexical_cast<std::string>((bool)xmlValue);
			//std::cout << "BoolValue: " << value << std::endl;
			ret = true;
			break;
		case XmlRpc::XmlRpcValue::TypeDateTime:
			// NOT IMPLEMENTED
			break;
		case XmlRpc::XmlRpcValue::TypeDouble:
			value = lexical_cast<std::string>((double)xmlValue);
			ret = true;
			break;
		case XmlRpc::XmlRpcValue::TypeInt:
			value = lexical_cast<std::string>((int)xmlValue);
			ret = true;
			break;
		case XmlRpc::XmlRpcValue::TypeString:
			value = (std::string)xmlValue;
			ret = true;
			break;
		case XmlRpc::XmlRpcValue::TypeStruct:
			// NOT IMPLEMENTED
			break;
		default:
			break;
	}


	return ret;
}

} // namespace
