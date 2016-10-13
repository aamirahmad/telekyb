/*
 * StateEstimatorControllerOptions.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <tk_state/StateEstimatorControllerOptions.hpp>

namespace TELEKYB_NAMESPACE {

StateEstimatorControllerOptions::StateEstimatorControllerOptions()
	: OptionContainer("StateEstimatorController")
{
	tPublisherTopic = addOption<std::string>("tPublisherTopic",
			"The Topic at which TKState's are published.", "TKState", false, true);
	tTransformStampedTopic = addOption<std::string>("tTransformStampedTopic",
			"The Topic at which geometry_msgs::TransformStamped are published.", "UAVTransform", false, true);
	tPluginLookupName = addOption<std::string>("tPluginLookupName",
			"Specifies the Plugin to use to generate TKState msgs", "tk_state/SSXStateEstimator", false, true);

	tPublishRosTransform = addOption<bool>("tPublishRosTransform",
			"Specifies if a transform should be published", false, false, false);

	tPublishRosTransformStamped = addOption<bool>("tPublishRosTransformStamped",
			"Specifies if a geometry_msgs::TransformStamped should be published", false, false, false);

    tTransformParentFrame= addOption<std::string>("tTransformParentID",
			"Parent Frame ID for Transform", "world", false, true);

    tTransformChildFrame = addOption<std::string>("tTransformChildFrame",
            "Parent Frame ID for Transform", "base_link", false, true);

}

}
