/*
 * TKTrajectory.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef TKTRAJECTORY_HPP_
#define TKTRAJECTORY_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>
#include <telekyb_base/Time/Time.hpp>
#include <telekyb_base/Spaces/R3.hpp>

#include <telekyb_msgs/TKTrajectory.h>

namespace TELEKYB_NAMESPACE {

class TKTrajectory {
public:
    // Position
    Position3D position;
    Velocity3D velocity;
    Acceleration3D acceleration;
    Eigen::Vector3d jerk;
    Eigen::Vector3d snap;

    PosControlType xAxisCtrl;
    PosControlType yAxisCtrl;
    PosControlType zAxisCtrl;

    // Output Regulator (Geometric tracking) for Attitude, added by Yuyi Liu
    RotAngle3D rotangle;
    AngVelocity3D angvelocity;
    PosControlType CtrlMode;


    // header attributes
    std::string frame_id;
    Time time;

    // Yaw
    double yawAngle;
    double yawRate;
    double yawAcceleration;
    YawControlType yawCtrl;

    TKTrajectory();
    TKTrajectory(const telekyb_msgs::TKTrajectory& tkTrajMsg);
    virtual ~TKTrajectory();

    void toTKTrajMsg(telekyb_msgs::TKTrajectory& tkTrajMsg) const;
    void fromTKTrajMsg(const telekyb_msgs::TKTrajectory& tkTrajMsg);

    void setHeader(
            const Time t = Time(ros::Time::now()),
            const std::string id = ""
            );
    // Convenience Setters

    void setPosition(
            const Position3D& position_,
            const Velocity3D& velocity_ = Velocity3D::Zero(),
            const Acceleration3D& acceleration_ = Acceleration3D::Zero(),
            const Eigen::Vector3d& jerk_ = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& snap_ = Eigen::Vector3d::Zero()
            );
    void setVelocity(
            const Velocity3D& velocity_,
            const Acceleration3D& acceleration_ = Acceleration3D::Zero(),
            const Eigen::Vector3d& jerk_ = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& snap_ = Eigen::Vector3d::Zero()
            );
    void setAcceleration(
            const Acceleration3D& acceleration_,
            const Eigen::Vector3d& jerk_ = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& snap_ = Eigen::Vector3d::Zero()
            );
    // Geometric tracking setter, added by Yuyi Liu
    void setGeometric(
            const Position3D& position_,
            const Velocity3D& velocity_ = Velocity3D::Zero(),
            const Acceleration3D& acceleration_ = Acceleration3D::Zero(),
            const Eigen::Vector3d& jerk_ = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& snap_ = Eigen::Vector3d::Zero(),
            const RotAngle3D& rotangle_ = Eigen::Vector3d::Zero(),
            const AngVelocity3D& angvelocity_ = Velocity3D::Zero()
            );

    void setYawAngle(
            double yawAngle_,
            double yawRate_ = 0.0,
            double yawAcceleration_ = 0.0
            );
    void setYawRate(
            double yawRate_,
            double yawAcceleration_ = 0.0
            );
    //	void setYawAcceleration(
    //			double yawAcceleration_
    //			);

    GlobalPosControlType getGlobalPositionControlType() const;
};

// Inline Functions
inline
void TKTrajectory::setPosition(
        const Position3D& position_,
        const Velocity3D& velocity_,
        const Acceleration3D& acceleration_,
        const Eigen::Vector3d& jerk_,
        const Eigen::Vector3d& snap_
        )
{
    position = position_;
    velocity = velocity_;
    acceleration = acceleration_;
    jerk = jerk_;
    snap = snap_;

    xAxisCtrl = PosControlType::Position;
    yAxisCtrl = PosControlType::Position;
    zAxisCtrl = PosControlType::Position;
}

inline
void TKTrajectory::setVelocity(
        const Velocity3D& velocity_,
        const Acceleration3D& acceleration_,
        const Eigen::Vector3d& jerk_,
        const Eigen::Vector3d& snap_
        )
{
    position = Position3D::Zero();
    velocity = velocity_;
    acceleration = acceleration_;
    jerk = jerk_;
    snap = snap_;

    xAxisCtrl = PosControlType::Velocity;
    yAxisCtrl = PosControlType::Velocity;
    zAxisCtrl = PosControlType::Velocity;
}

inline
void TKTrajectory::setHeader(const Time t, const std::string id){
    time = t;
    frame_id = id;
}


inline
void TKTrajectory::setAcceleration(
        const Acceleration3D& acceleration_,
        const Eigen::Vector3d& jerk_,
        const Eigen::Vector3d& snap_
        )
{
    position = Position3D::Zero();
    velocity = Velocity3D::Zero();
    acceleration = acceleration_;
    jerk = jerk_;
    snap = snap_;

    xAxisCtrl = PosControlType::Acceleration;
    yAxisCtrl = PosControlType::Acceleration;
    zAxisCtrl = PosControlType::Acceleration;
}
// Geometric tracking setter, added by Yuyi Liu
inline
void TKTrajectory::setGeometric(
        const Position3D& position_,
        const Velocity3D& velocity_,
        const Acceleration3D& acceleration_,
        const Eigen::Vector3d& jerk_,
        const Eigen::Vector3d& snap_,
        const RotAngle3D& rotangle_,
        const AngVelocity3D& angvelocity_
        )
{
    position = position_;
    velocity = velocity_;
    acceleration = acceleration_;
    jerk = jerk_;
    snap = snap_;
    rotangle = rotangle_;
    angvelocity = angvelocity_;

    CtrlMode = PosControlType::Geometric;
}

inline
void TKTrajectory::setYawAngle(
        double yawAngle_,
        double yawRate_,
        double yawAcceleration_
        )
{
    yawAngle = yawAngle_;
    yawRate = yawRate_;
    yawAcceleration = yawAcceleration_;

    yawCtrl = YawControlType::AngleOnBoard;
}

inline
void TKTrajectory::setYawRate(
        double yawRate_,
        double yawAcceleration_
        )
{
    yawAngle = 0.0;
    yawRate = yawRate_;
    yawAcceleration = yawAcceleration_;

    yawCtrl = YawControlType::RateOnBoard;
}

//inline
//void TKTrajectory::setYawAcceleration(
//		double yawAcceleration_
//		)
//{
//
//}

inline
GlobalPosControlType TKTrajectory::getGlobalPositionControlType() const
{
    GlobalPosControlType retValue = GlobalPosControlType::Mixed;

    if (xAxisCtrl == PosControlType::Position
            && yAxisCtrl == PosControlType::Position
            && zAxisCtrl == PosControlType::Position) {
        retValue = GlobalPosControlType::Position;
    } else if (xAxisCtrl == PosControlType::Velocity
               && yAxisCtrl == PosControlType::Velocity
               && zAxisCtrl == PosControlType::Velocity) {
        retValue = GlobalPosControlType::Velocity;
    } else if (xAxisCtrl == PosControlType::Acceleration
               && yAxisCtrl == PosControlType::Acceleration
               && zAxisCtrl == PosControlType::Acceleration) {
        retValue = GlobalPosControlType::Acceleration;
    } else if (CtrlMode == PosControlType::Geometric) {
        retValue = GlobalPosControlType::Geometric;         //Geometric tracking setter, added by Yuyi Liu
    } else 	{
        // Mixed
    }


    return retValue;
}


// Delegate Definition
//class TKTrajInputReceiver {
//public:
//	virtual ~TKTrajInputReceiver() {}
//	virtual void trajInputStep(const TKTrajectory& input) = 0;
//};

}

#endif /* TKTRAJECTORY_HPP_ */
