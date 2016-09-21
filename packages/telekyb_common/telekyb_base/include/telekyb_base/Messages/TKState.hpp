/*
* TKState.hpp
*
*  Created on: Nov 8, 2011
*      Author: mriedel
*/

#ifndef TKSTATE_HPP_
#define TKSTATE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Spaces/R3.hpp>
#include <telekyb_msgs/TKState.h>

#include <telekyb_base/Time/Time.hpp>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

namespace TELEKYB_NAMESPACE {

class TKState {
public:
	Time time;
    std::string frame_id;
	Position3D position; // (x,y,z)
	Quaternion orientation; // (w,x,y,z)
	Velocity3D linVelocity; // (x,y,z)
	Velocity3D angVelocity; // (omega_x, omega_y, omega_z)

	TKState();
	TKState(const telekyb_msgs::TKState& tkStateMsg);
	virtual ~TKState();

	void toTKStateMsg(telekyb_msgs::TKState& tkStateMsg) const;
	void fromTKStateMsg(const telekyb_msgs::TKState& tkStateMsg);
    void toROSGeometryMsgPoseStamped(geometry_msgs::PoseStamped& msg) const;

	Vector3D getEulerRPY() const;
	Vector3D getEulerDotRPY() const;

};

}

#endif /* TKSTATE_HPP_ */
