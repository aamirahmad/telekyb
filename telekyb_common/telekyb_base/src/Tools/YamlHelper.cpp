/*
 * YamlHelper.cpp
 *
 *  Created on: Nov 13, 2011
 *      Author: mriedel
 */


#include <telekyb_base/Tools/YamlHelper.hpp>

namespace TELEKYB_NAMESPACE
{

bool YamlHelper::parseNodeToString(const YAML::Node& node, std::string& output) {
	YAML::Emitter emit;
	emit.SetMapFormat(YAML::Flow); // also flow
	emit.SetSeqFormat(YAML::Flow); // Flow format.
	// should I catch something here?
	emit << node;

	output = emit.c_str();
	return true;
}

bool YamlHelper::parseStringToNode(const std::string& string, YAML::Node& node) {
	// catch something?
	node = YAML::Load(string);
	return true;
}

// Not needed anymore.
//bool YamlHelper::getFirstNodeFromString(const std::string& input, YAML::Node& output)
//{
//
//	return true;
//}

}
