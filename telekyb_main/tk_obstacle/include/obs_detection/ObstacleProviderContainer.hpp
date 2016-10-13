/*
 * ObstacleProviderContainer.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef OBSTACLEPROVIDERCONTAINER_HPP_
#define OBSTACLEPROVIDERCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <obs_detection/ObstacleProvider.hpp>
#include <pluginlib/class_loader.h>

// stl
#include <set>

namespace TELEKYB_NAMESPACE {

class ObstacleProviderContainer {
protected:
	pluginlib::ClassLoader<ObstacleProvider> obstacleProviderLoader;
	// Behaviorset. Contains all loaded Behaviors from all Containers!
	std::set<ObstacleProvider*> obstacleProviderInstances;

public:
	ObstacleProviderContainer();
	virtual ~ObstacleProviderContainer();

	// should be const, but getDeclaredClasses is declared wrongly.
	void getAvailableObstacleProviders(std::vector<std::string>& obstacleProviderClassNames);

	// returns 0 if fails
	ObstacleProvider* loadObstacleProvider(const std::string& obstacleProviderClassName);
	void unLoadObstacleProvider(ObstacleProvider* tm);

	const std::set<ObstacleProvider*>& getLoadedObstacleProviders() const;

};

} /* namespace telekyb */
#endif /* OBSTACLEPROVIDERCONTAINER_HPP_ */
