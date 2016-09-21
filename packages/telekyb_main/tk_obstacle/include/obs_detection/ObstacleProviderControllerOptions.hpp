/*
 * ObstacleProviderControllerOptions.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef OBSTACLEPROVIDERCONTROLLEROPTIONS_HPP_
#define OBSTACLEPROVIDERCONTROLLEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class ObstacleProviderControllerOptions : public OptionContainer {
public:
	Option<double>* tObsSpinrate;
	Option< std::vector<std::string> >* tObstacleProviders;
	Option< std::string >* tTKStateTopicName;
	Option< double >* tInitialStateTimeout;
	Option< std::string >* tObsPubTopicName;
	ObstacleProviderControllerOptions();
};

} /* namespace telekyb */
#endif /* OBSTACLEPROVIDERCONTROLLEROPTIONS_HPP_ */
