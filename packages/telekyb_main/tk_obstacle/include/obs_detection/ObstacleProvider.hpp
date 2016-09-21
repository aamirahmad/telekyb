/*
 * ObstacleProvider.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef OBSTACLEPROVIDER_HPP_
#define OBSTACLEPROVIDER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Spaces.hpp>

#include <telekyb_base/Messages.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// stl
#include <string>
#include <vector>

namespace TELEKYB_NAMESPACE {

class ObstacleProvider {
protected:
	std::string name;
	ObstacleProvider(const std::string& name_);

public:
	virtual ~ObstacleProvider();
	std::string getName() const;

	// called directly after Creation
	virtual void initialize() = 0;

	// called right before destruction
	virtual void destroy() = 0;

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const = 0;
};

} /* namespace telekyb */
#endif /* OBSTACLEPROVIDER_HPP_ */
