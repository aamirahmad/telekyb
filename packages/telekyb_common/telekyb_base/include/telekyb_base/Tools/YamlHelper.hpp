/*
 * YamlHelper.hpp
 *
 *  Created on: Nov 13, 2011
 *      Author: mriedel
 */

#ifndef YAMLHELPER_HPP_
#define YAMLHELPER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <ros/console.h>

#include <yaml-cpp/yaml.h>
// Conversion
#include <telekyb_base/Tools/YamlConversion.hpp>

namespace TELEKYB_NAMESPACE
{


class YamlHelper {
public:
	// Parse Node to String
	static bool parseNodeToString(const YAML::Node& node, std::string& output);
	static bool parseStringToNode(const std::string& string, YAML::Node& node);

	// Parse Node to Value
	template < class _T >
	static bool parseNodeToValue(const YAML::Node& input, _T& value) {
		bool success = true;
		try {
			value = input.as<_T>();
			//input >> value;
		} catch (YAML::Exception &e) {
			ROS_ERROR_STREAM("Error converting from YAML! " << e.what());
			success = false;
		} catch (std::runtime_error &e) {
			ROS_ERROR_STREAM("Error converting from YAML! " << e.what());
			success = false;
		}

		return success;
	}

	// Template Helpers
	template < class _T >
	static bool parseStringToValue(const std::string& input, _T& value) {
		//ROS_INFO_STREAM("Parsing: " << input);
		YAML::Node node = YAML::Load(input);
		return parseNodeToValue(node, value);
	}

	template < class _T >
	static bool parseValueToString(const _T& value , std::string& output) {
		YAML::Node node( value );// = value;
		return parseNodeToString(node, output);
	}

	// direct accessor
	template < class _T >
	static std::string parseValueToString(const _T& value) {
		std::string str("yaml-conversion failed");
		parseValueToString(value, str);
		return str;
	}



};

}


#endif /* YAMLHELPER_HPP_ */
