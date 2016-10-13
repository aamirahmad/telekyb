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
// Contact info: antonio.franchi@tuebingen.mpg.de stegagno@dis.uniroma1.it
//
// ----------------------------------------------------------------------------

#include <telekyb_base/Options/RawOptionsContainer.hpp>

#include <telekyb_base/File/File.hpp>
#include <telekyb_base/Tools/XmlRpcHelper.hpp>

// ros
#include <ros/console.h>
#include <ros/assert.h>

namespace TELEKYB_NAMESPACE
{

// private
// definition
StringMap RawOptionsContainer::rawOptionsMap;

bool RawOptionsContainer::addUpdateDashOption(std::string dashkey, const std::string& value, bool overwrite, bool showDashError)
{
	if (dashkey.length() < 2 || !(dashkey[0] == '-' && dashkey[1] == '-'))
	{
		if (showDashError)
		{
			ROS_WARN_STREAM("Syntax error:" << dashkey << " must be preceded by a dash '--'.");
		}
		return false;
	}

	// Syntax ok
	dashkey.erase(0,2);
	bool ret = true;

	if (overwrite)
		addUpdateOption(dashkey,value);
	else
		ret = addOption(dashkey,value);

	return ret;
}


// public
bool RawOptionsContainer::hasOption(const std::string& key)
{
	StringMap::iterator it;
	it = rawOptionsMap.find(key);

	return (it != rawOptionsMap.end());
}

bool RawOptionsContainer::removeOption(const std::string& key)
{
	// erase return 1 if Object was erased.
	return (rawOptionsMap.erase(key) == 1);
}

bool RawOptionsContainer::getOption(const std::string& key, std::string& value)
{
	bool ret = false;

	StringMap::iterator it;
	it = rawOptionsMap.find(key);

	if (it != rawOptionsMap.end())
	{
		value = it->second;
		ret = true;
	}

	return ret;
}
// Option must exist
bool RawOptionsContainer::updateOption(const std::string& key, const std::string& value)
{
	bool ret = false;
	// would be more efficient with iterator. Doesn't matter.
	if (hasOption(key))
	{
		rawOptionsMap[key] = value;
		ret = true;

		ROS_DEBUG_STREAM("Updated: " << key << " with value " <<  value);
	}
	else
	{
		ROS_WARN_STREAM("Saw: " << key << " but did NOT update, because option was not found!");
	}
	return ret;
}
// Option must NOT exist
bool RawOptionsContainer::addOption(const std::string& key, const std::string& value)
{
	bool ret = false;
	// would be more efficient with iterator. Doesn't matter.
	if (!hasOption(key))
	{
		rawOptionsMap[key] = value;
		ret = true;

		ROS_DEBUG_STREAM("Added: " << key << " with value " << value);
	}
	else
	{
		ROS_WARN_STREAM("Saw: " << key << " but did NOT add, because it is already set.!");
	}
	return ret;
}
// Value gets set whether option exists or not
void RawOptionsContainer::addUpdateOption(const std::string& key, const std::string& value)
{
	rawOptionsMap[key] = value;

	ROS_DEBUG_STREAM("Added/Updated: " << key << " with value " << value);
}

// parse CommandLine Arguments.
bool RawOptionsContainer::parseCommandLine(int argc, char* const argv[], bool overwrite )
{
	if (argc % 2 == 0)
	{
		ROS_FATAL_STREAM("Non-odd options, " << TELEKYB_BASENAME << " launching aborted.");
		//ROS_BREAK();
		ros::shutdown();
		return false;
	}

	// ok to continue
	bool ret = true;
	for (int i = 1; i < argc; i += 2)
	{
		std::string name(argv[i]);
		std::string value(argv[i + 1]);

		ret = addUpdateDashOption(name,value,overwrite,true);
	}

	return ret;
}

bool RawOptionsContainer::parseCommandLine(const std::vector<std::string>& commandLineArgs, bool overwrite )
{
	if (commandLineArgs.size() % 2 == 0)
	{
		ROS_FATAL_STREAM("Non-odd options, " << TELEKYB_BASENAME << " launching aborted.");
		//ROS_BREAK();
		ros::shutdown();
		return false;
	}

	bool ret = true;
	for(unsigned int i = 0; i < commandLineArgs.size(); i+=2)
	{
		// Function checks for correct syntax // last argument false: report no errors!
		ret = addUpdateDashOption(commandLineArgs.at(i),commandLineArgs.at(i+1), overwrite, true);
	}
	return ret;
}

bool RawOptionsContainer::parseFile(const std::string& fileName, bool overwrite)
{
	bool ret = true;

	File file(fileName);
	std::vector<std::string> words;
	ret = file.getWords(words);

	int nWords = words.size();
	// only walk to the second to last, because the last field must always be an option.
	for(int i = 0; i < nWords-1; i++)
	{
		// Function checks for correct syntax // last argument false: report no errors!
		addUpdateDashOption(words[i],words[i+1], overwrite, false);
	}

	return ret;
}

// be careful with this.
void RawOptionsContainer::clearOptions()
{
	rawOptionsMap.clear();
}

void RawOptionsContainer::print()
{
	std::cout << toString();
}
std::string RawOptionsContainer::toString()
{
	std::stringstream ss;
	StringMap::iterator it;
	for (it = rawOptionsMap.begin(); it != rawOptionsMap.end(); it++)
	{
		ss << "-" << it->first << " " << it->second;
		ss << "\n";
	}

	return ss.str();
}

StringMap& RawOptionsContainer::getMap()
{
	return rawOptionsMap;
}

} // namespace


