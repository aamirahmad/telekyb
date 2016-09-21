/*
 * ExternalObsPoint.hpp
 *
 *  Created on: Dec 16, 2011
 *      Author: mriedel
 */

#ifndef EXTERNALOBSPOINT_HPP_
#define EXTERNALOBSPOINT_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <obs_detection/ObstacleProvider.hpp>
#include <telekyb_base/Options.hpp>
#include <geometry_msgs/PoseStamped.h>
//#include <boost/thread/mutex.hpp>
#include <telekyb_base/ROS/GenericSubscriber.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_obs {

class ExternalObsPointOptions : public OptionContainer {
public:
	Option< std::vector<std::string> >* tObsTransformTopicNames;
	Option< std::vector<double> >* tObsRadius;
	Option<Eigen::Matrix3d>* tObsTranformConvMatrix;
	ExternalObsPointOptions();
};

class ExternalObsPoint : public ObstacleProvider {
protected:
	ExternalObsPointOptions options;
//	ros::Subscriber obsTranformSub;
	std::vector<std::string> transformTopicNames;
	std::vector<double> obstacleRadii;
	std::vector< GenericSubscriber<geometry_msgs::PoseStamped>* > subscribers;

//	Position3D obsPoint;
//	mutable boost::mutex obsPointMutex;

public:
	ExternalObsPoint();
	virtual ~ExternalObsPoint();

	// called directly after Creation
	virtual void initialize();

	// called right before destruction
	virtual void destroy();

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const;

	// TransformCB
//	void obsPointCB(const geometry_msgs::TransformStamped::ConstPtr& msg);
};

} /* namespace telekyb_obs */
#endif /* EXTERNALOBSPOINT_HPP_ */
