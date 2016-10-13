/*
 * mpcPathFollow.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: eruff
 */

#ifndef FIXED_POINT_HOVER_HPP_
#define FIXED_POINT_HOVER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/Behavior.hpp>

// Options
#include <telekyb_base/Options.hpp>
// sensormsgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// telekyb
#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// boost
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

// eigen for tranformation
#include <eigen3/Eigen/Geometry>

// cpp
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
// mpc
extern "C" {
#include <muaoMPC/cmpc/include/mpc.h>  /* the auto-generated code */
}

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {



class mpcPathFollow : public Behavior {
protected:
    // Option
    Option<std::string>* tmpcPathFollowPathTopic;

    // ROS
    ros::NodeHandle nodeHandle;
    ros::Subscriber _pathSub;
    ros::Publisher _x_refPub;
    ros::Publisher _x_predictPub;

    tf::TransformListener _tfListener;

    void pathCB(const nav_msgs::PathConstPtr& path);

    void updateVelocity(const TKState& currentState, TKTrajectory& generatedTrajInput);

    void buildXref(real_t *currentState, real_t* x_ref,const nav_msgs::Path& path);
    void findClosestWayPoint(real_t *&state, const nav_msgs::Path& path);

    // Data
    double _yawRate;
    nav_msgs::Path _currentPath;
    bool _currentPathCalledBackOnce;
    bool _writtenPathOnce;
    geometry_msgs::Pose _x_predict;
    // nearest node on the path to warmstart search for next 3N
    int _k;
    // Init Service
    //ros::ServiceServer initService;

    bool valid;
public:
    mpcPathFollow();

    // from BehaviorInterface
    virtual void initialize();
    virtual void destroy();

    // Called directly after Change Event is registered.
    virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
    // Called after actual Switch. Note: During execution trajectoryStepCreation is used
    virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
    // Called directly after Change Event is registered: During execution trajectoryStepTermination is used
    virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
    // Called after actual Switch. Runs in seperate Thread.
    virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

    // called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
    virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

    // called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or mpcPathFollowHover if undef).
    virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

    // called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
    virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

    // Return true if the active Behavior is (still) valid. Initiate Switch otherwise
    virtual bool isValid(const TKState& currentState) const;

    virtual void setTrajectoryHeader(TKTrajectory& generatedTrajInput);

};

}

#endif /* HOVERBEHAVIOR_HPP_ */
