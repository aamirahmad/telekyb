/*
 * ExternalObsPoint.cpp
 *
 *  Created on: Dec 16, 2011
 *      Author: mriedel
 */

#include <obs_providers/ExternalObsPoint.hpp>

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_obs::ExternalObsPoint, TELEKYB_NAMESPACE::ObstacleProvider);

namespace telekyb_obs {

ExternalObsPointOptions::ExternalObsPointOptions()
	: OptionContainer("ExternalObsPoint")
{
	tObsTransformTopicNames = addOption< std::vector<std::string> >("tObsTransformTopicNames",
			"TopicName of Obstacle Transform Topic",std::vector<std::string>(),true,true);
	tObsRadius = addOption< std::vector<double> >("tObsRadius",
			"TopicName of Obstacle Transform Topic",std::vector<double>(),false,true);

	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tObsTranformConvMatrix = addOption<Eigen::Matrix3d>("tObsTranformConvMatrix",
			"Conversion Matrix applied to TransformPoint. (Def: Vicon to NED)", m,false,true);
}

ExternalObsPoint::ExternalObsPoint()
	: ObstacleProvider("tk_obstacle/ExternalObsPoint")
{

}

ExternalObsPoint::~ExternalObsPoint()
{
	// free
	for (unsigned int i = 0; i < subscribers.size(); ++i) {
		delete subscribers[i];
	}
}

// called directly after Creation
void ExternalObsPoint::initialize()
{
	ros::NodeHandle nodeHandle(ROSModule::Instance().getMainNodeHandle());
	transformTopicNames = options.tObsTransformTopicNames->getValue();

	obstacleRadii = options.tObsRadius->getValue();
	if (obstacleRadii.size() != transformTopicNames.size()) {
		ROS_WARN_STREAM(options.tObsRadius->getNSName() << " length not equal to "
				<< options.tObsTransformTopicNames->getNSName() << "! Adjusting!");
		obstacleRadii.resize(transformTopicNames.size(), 0.0);
	}

	subscribers.resize(transformTopicNames.size());

	for (unsigned int i = 0; i < transformTopicNames.size(); ++i) {
		ROS_WARN_STREAM("Subscribing to " << transformTopicNames[i]);
		subscribers[i] = new GenericSubscriber<geometry_msgs::PoseStamped>(nodeHandle, transformTopicNames[i],1);
	}

//	obsTranformSub = nodeHandle.subscribe<geometry_msgs::TransformStamped>(
//			options.tObsTransformTopicName->getValue(),1, &ExternalObsPoint::obsPointCB, this);
}

// called right before destruction
void ExternalObsPoint::destroy()
{
//	obsTranformSub.shutdown();
}


void ExternalObsPoint::getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const
{
	for (unsigned int i = 0; i < subscribers.size(); ++i) {
		geometry_msgs::PoseStamped msg = subscribers[i]->getLastMsg();
		Eigen::Vector3d obsPoint(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
		// Convert.
		obsPoint = options.tObsTranformConvMatrix->getValue() * obsPoint;

		if (obstacleRadii[i] > 0.0) {
			// Adjust point
			Eigen::Vector3d obsVector = lastState.position - obsPoint;

			if (obsVector.norm() < obstacleRadii[i]) {
				ROS_WARN("Object inside Obstaclesphere!!!");
				obstaclePoints.push_back(obsPoint + obsVector * 0.9); // go 10% deeper.
			} else {
				// Point on Sphere
				obstaclePoints.push_back(obsPoint + obsVector.normalized() * obstacleRadii[i]);
			}
		} else {
			obstaclePoints.push_back(obsPoint);
		}
	}
}

//void ExternalObsPoint::obsPointCB(const geometry_msgs::TransformStamped::ConstPtr& msg)
//{
//	Position3D rawPosition;
//	rawPosition(0) = msg->transform.translation.x;
//	rawPosition(1) = msg->transform.translation.y;
//	rawPosition(2) = msg->transform.translation.z;
//
//	boost::mutex::scoped_lock obsPointLock(obsPointMutex);
//	obsPoint = options.tObsTranformConvMatrix->getValue() * rawPosition;
//}

} /* namespace telekyb_obs */
