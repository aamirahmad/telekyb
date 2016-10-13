/*
 * ObstacleProvider.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_detection/ObstacleProvider.hpp>

namespace TELEKYB_NAMESPACE {

ObstacleProvider::ObstacleProvider(const std::string& name_)
	: name(name_)
{

}

ObstacleProvider::~ObstacleProvider()
{

}


std::string ObstacleProvider::getName() const
{
	return name;
}

} /* namespace telekyb */
