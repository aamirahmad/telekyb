/*
 * TKState.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Messages/TKState.hpp>

namespace TELEKYB_NAMESPACE {

TKState::TKState()
    : time(ros::Time::now()),
      position(0.0,0.0,0.0),
      orientation(1.0,0.0,0.0,0.0),
      linVelocity(0.0,0.0,0.0),
      angVelocity(0.0,0.0,0.0),
      frame_id("")
{
	// TODO Auto-generated constructor stub

}

TKState::TKState(const telekyb_msgs::TKState& tkStateMsg)
{
	fromTKStateMsg(tkStateMsg);
}

TKState::~TKState() {
	// TODO Auto-generated destructor stub
}

void TKState::toTKStateMsg(telekyb_msgs::TKState& tkStateMsg) const
{
	tkStateMsg.header.stamp = time.toRosTime();
    tkStateMsg.header.frame_id = frame_id;
	// Position
	tkStateMsg.pose.position.x = position(0);
	tkStateMsg.pose.position.y = position(1);
	tkStateMsg.pose.position.z = position(2);

	// Orientation
	tkStateMsg.pose.orientation.w = orientation.w();
	tkStateMsg.pose.orientation.x = orientation.x();
	tkStateMsg.pose.orientation.y = orientation.y();
	tkStateMsg.pose.orientation.z = orientation.z();

	// linear Velocity
	tkStateMsg.twist.linear.x = linVelocity(0);
	tkStateMsg.twist.linear.y = linVelocity(1);
	tkStateMsg.twist.linear.z = linVelocity(2);

	// angular Velocity
	tkStateMsg.twist.angular.x = angVelocity(0);
	tkStateMsg.twist.angular.y = angVelocity(1);
	tkStateMsg.twist.angular.z = angVelocity(2);

}

void TKState::toROSGeometryMsgPoseStamped(geometry_msgs::PoseStamped& msg) const
{

    msg.header.stamp = time.toRosTime();
    msg.header.frame_id = frame_id;
    // Position
    msg.pose.position.x = position(0);
    msg.pose.position.y = position(1);
    msg.pose.position.z = position(2);

    // Orientation
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

}

void TKState::fromTKStateMsg(const telekyb_msgs::TKState& tkStateMsg)
{
	time = Time(tkStateMsg.header.stamp);
    frame_id = tkStateMsg.header.frame_id;
	// Position
	position(0) = tkStateMsg.pose.position.x;
	position(1) = tkStateMsg.pose.position.y;
	position(2) = tkStateMsg.pose.position.z;

	// Orientation
	orientation.w() = tkStateMsg.pose.orientation.w;
	orientation.x() = tkStateMsg.pose.orientation.x;
	orientation.y() = tkStateMsg.pose.orientation.y;
	orientation.z() = tkStateMsg.pose.orientation.z;
	orientation.normalize();

	// linear Velocity
	linVelocity(0) = tkStateMsg.twist.linear.x;
	linVelocity(1) = tkStateMsg.twist.linear.y;
	linVelocity(2) = tkStateMsg.twist.linear.z;

	// angular Velocity
	angVelocity(0) = tkStateMsg.twist.angular.x;
	angVelocity(1) = tkStateMsg.twist.angular.y;
	angVelocity(2) = tkStateMsg.twist.angular.z;
}

Vector3D TKState::getEulerRPY() const
{
	//XXX:the code below seems to be broken
	//return orientation.toRotationMatrix().eulerAngles(0,1,2); // xyz

	//Debugging:
//	Vector3D curOrientation = orientation.toRotationMatrix().eulerAngles(0,1,2);
//	std::cout << "quaternion: " << orientation.w() << " " << orientation.x() << " " << orientation.y() << " " << orientation.z() << std::endl;

	Vector3D curOrientation;

	double q0 = orientation.w();
	double q1 = orientation.x();
	double q2 = orientation.y();
	double q3 = orientation.z();
	curOrientation(0) = std::atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	curOrientation(1) = -std::asin(2.0f * (q1 * q3 - q0 * q2));
	curOrientation(2) = std::atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

//	std::cout << "euler: " << curOrientation(0) << " " << curOrientation(1) << " " << curOrientation(2) << std::endl;

	return curOrientation;

}

Vector3D TKState::getEulerDotRPY() const
{
//	Vector3D eulerdot;
	Vector3D rpy = getEulerRPY();

	Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

	T(0,1) = tan((double)rpy(1))*sin((double)rpy(0));
	T(0,2) = tan((double)rpy(1))*cos((double)rpy(0));
	T(1,1) = cos((double)rpy(0));
	T(1,2) = -sin((double)rpy(0));
	T(2,1) = sin((double)rpy(0))/cos((double)rpy(1));
	T(2,2) = cos((double)rpy(0))/cos((double)rpy(1));

//	eulerdot = T*angVelocity;
//	Matrix4x4 T = Matrix4x4.identity;
//			T.m01 = Mathf.Tan(rpy.y)*Mathf.Sin(rpy.x);
//			T.m02 = Mathf.Tan(rpy.y)*Mathf.Cos(rpy.x);
//			T.m11 = Mathf.Cos(rpy.x);
//			T.m12 = -Mathf.Sin(rpy.x);
//			T.m21 = Mathf.Sin(rpy.x)/Mathf.Cos(rpy.y);
//			T.m22 = Mathf.Cos(rpy.x)/Mathf.Cos(rpy.y);
//	Vector3 rpy_dot = T.MultiplyVector(localOmega);
	return T*angVelocity;
}

}

