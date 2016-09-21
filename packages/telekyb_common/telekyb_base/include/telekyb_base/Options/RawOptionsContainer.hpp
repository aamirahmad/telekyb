// ----------------------------------------------------------------------------
//
// $Id$
//
// Copyright 2008, 2009, 2010, 2011  Antonio Franchi and Paolo Stegagno
//
// This file is part of MIP.
//
// MIP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MIP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MIP. If not, see <http://www.gnu.org/licenses/>.
//
// Contact info: martin.riedel@tuebingen.mpg.de
//
// ----------------------------------------------------------------------------


#ifndef RAWOPTIONSCONTAINER_HPP__
#define RAWOPTIONSCONTAINER_HPP__

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Tools/XmlRpcHelper.hpp>

// STL
#include <map>
#include <string>
#include <iostream>
#include <sstream>

// YAML
#include <yaml-cpp/yaml.h>
// Helper
#include <telekyb_base/Tools/YamlHelper.hpp>

// ros
#include <ros/ros.h>

namespace TELEKYB_NAMESPACE
{

typedef std::map<std::string, std::string> StringMap;

class RawOptionsContainer {
protected:
	static StringMap rawOptionsMap;
	static bool addUpdateDashOption(std::string dashkey, const std::string& value, bool overwrite, bool showDashError = false);

public:
	// Checking
	static bool hasOption(const std::string& key);
	// Manipulation
	static bool removeOption(const std::string& key);
	static bool getOption(const std::string& key, std::string& value);
	// Option must exist
	static bool updateOption(const std::string& key, const std::string& value);
	// Option must NOT exist
	static bool addOption(const std::string& key, const std::string& value);
	// Value gets set whether option exists or not
	static void addUpdateOption(const std::string& key, const std::string& value);

	// parse CommandLine Arguments.
	static bool parseCommandLine(int argc, char* const argv[], bool overwrite = false);
	static bool parseCommandLine(const std::vector<std::string>& commandLineArgs, bool overwrite = false);

	// parse File
	static bool parseFile(const std::string& fileName, bool overwrite = false);

	// Template Function for Object
	template < class _T > //, class StringConversion_>
	static bool getOptionValue(const std::string& name, _T& value) {
		bool success = false;
		std::string stringValue;
		if (getOption(name, stringValue)) {
			success = true;
		} else if (ros::param::has("~" + name)) {
			//ROS_INFO_STREAM("Found Option " << name << " in private Nodehandle ParameterServer");
			XmlRpc::XmlRpcValue xmlValue;
			ros::param::get("~" + name, xmlValue);
			success = XmlRpcHelper::xmlRpcValuetoString(xmlValue, stringValue);
		} else {
			// success = false
		}

		if (success) {
			// remove from all containers!
			//removeOption(name);
			//ros::param::del("~" + name);

			if (! YamlHelper::parseStringToValue(stringValue, value) ) {
				ROS_ERROR_STREAM("Option " << name << ". Unable to parse String!");
				success = false;
			}
			//std::cout << "Parsed Value: " << value << std::endl;
		}
//		else {
//			ROS_DEBUG_STREAM("Option " << name << " not specified!");
//		}

		return success;
	}

//	template < class _T >
//	static bool getOptionValue(const std::string& name, _T& value) {
//		return getOptionValue< _T, StringConversion<_T> >(name, value);
//	}

	// be careful with this.
	static void clearOptions();

	static void print();
	static std::string toString();

	// Get Reference to StringMap
	static StringMap& getMap();

};

} // namespace

#endif /* OPTIONSCONTAINER_H_ */
