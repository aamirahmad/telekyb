/*
 * mpcPathFollowBehavior.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include "mpcPathFollow.hpp"
#include <telekyb_base/ROS.hpp>

#define MAX_INITIAL_VELOCITY 0.05


// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::mpcPathFollow, TELEKYB_NAMESPACE::Behavior)

extern struct mpc_ctl ctl; // already defined

namespace telekyb_behavior {

enum COMMAND_MAPPING{
    COMMAND_THRUST = 1,
    COMMAND_LEFTRIGHT = 3,
    COMMAND_FORWARDBACK = 4,
    COMMAND_YAW = 0,
    COMMAND_DISABLE = 5 // Logitech Joypad RB
};

mpcPathFollow::mpcPathFollow()
    : Behavior("tk_behavior/mpcPathFollow",  BehaviorType::Air),
      nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
      tmpcPathFollowPathTopic(NULL)
{
    valid = false;

}

void mpcPathFollow::initialize()
{
    /*
     *  set logger level
     */
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    tmpcPathFollowPathTopic = addOption<std::string>("tmpcPathFollowPathTopic",
                                                     "PathTopic","/path/Path",false,false);

    //options = new mpcPathFollowOptions(this);
    // can work with default parameters
    parameterInitialized = true;
}

void mpcPathFollow::destroy()
{

}

void mpcPathFollow::pathCB(const nav_msgs::PathConstPtr &path){


    ROS_WARN_STREAM("recieved new path");
    _currentPath = *path;
    _currentPathCalledBackOnce = true;
    _k = 0;
}

bool mpcPathFollow::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
    _pathSub = nodeHandle.subscribe<nav_msgs::Path>(tmpcPathFollowPathTopic->getValue(), 1,
                                                    boost::bind(&mpcPathFollow::pathCB, this,_1));

    _x_refPub = nodeHandle.advertise<geometry_msgs::PoseArray>("/x_ref",1);
    _x_predictPub = nodeHandle.advertise<geometry_msgs::PoseArray>("/x_predict",1);
    _currentPathCalledBackOnce = false;
    _writtenPathOnce = false;
    _x_predict.position.x = 0;
    _x_predict.position.y = 0;
    _x_predict.position.z = 0;
    _x_predict.orientation = tf::createQuaternionMsgFromYaw(0.0);
    return true;
}

void mpcPathFollow::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
    // not used
    //	ROS_ERROR("Sleeping after Switch to mpcPathFollow.");
    //	usleep(2*1000*1000); // 10 sec
    // configure the mpc
    ctl.conf->in_iter = 10;
    ctl.conf->ex_iter = 10;
    ctl.conf->warmstart = true;
}

void mpcPathFollow::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
    _pathSub.shutdown();
    _x_refPub.shutdown();
    _x_predictPub.shutdown();

}

void mpcPathFollow::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{

}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void mpcPathFollow::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    // Same as active
    this->trajectoryStepActive(currentState,generatedTrajInput);
}


void mpcPathFollow::findClosestWayPoint(real_t* &state, const nav_msgs::Path &path){

    double dist = INFINITY;
    unsigned int currentIdx = 0;
    //    ROS_WARN_STREAM("_k " << _k);
    for(size_t k = 0; k < path.poses.size(); k++){
        double tmpDist = sqrt(pow(state[0]-path.poses[k].pose.position.x,2)
                +pow(state[1]-path.poses[k].pose.position.y,2)
                +pow(state[2]-path.poses[k].pose.position.z,2));
        //ROS_WARN_STREAM("tmp dist " << tmpDist << " dist " << dist << " k " << _k);
        if(dist > tmpDist){
            dist = tmpDist;
            currentIdx = k;
        }
    }
    _k = currentIdx;

}


void mpcPathFollow::buildXref(real_t* currentState, real_t* x_ref, const nav_msgs::Path &path){



    this->findClosestWayPoint(currentState,path);
    //ROS_DEBUG_STREAM("nearest node on the path " << _k << " path length " << _currentPath.poses.size());

    int idx2 = _k+(4*1)+(MPC_STATES*4);
    if(idx2 > path.poses.size()-1){
        idx2 = path.poses.size()-1;
    }


    for(size_t i = 0; i < MPC_HOR; i++){
        int idx = _k+(15*i)+(5);
        if(idx > path.poses.size()-1){
            idx = path.poses.size()-1;
        }
        x_ref[MPC_STATES*i] = path.poses[idx].pose.position.x;
        x_ref[MPC_STATES*i+1] = path.poses[idx].pose.position.y;
        x_ref[MPC_STATES*i+2] = path.poses[idx].pose.position.z;
        if(MPC_STATES == 4){
            x_ref[MPC_STATES*i+3] = tf::getYaw(path.poses[idx].pose.orientation);
        }

    }

}

void mpcPathFollow::updateVelocity(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    // if there is a transformation between the Trajectory and the currentState
    //



}

void mpcPathFollow::setTrajectoryHeader(TKTrajectory& generatedTrajInput){
    generatedTrajInput.setHeader(ros::Time::now(),"/world");
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or mpcPathFollow if undef).
void mpcPathFollow::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    if(_currentPathCalledBackOnce){
        clock_t total_begin = clock();
        ros::Time start_time = ros::Time::now();
        real_t x[MPC_STATES];  /* current state of the system */
        real_t u_ref[MPC_HOR_INPUTS];  /* the input reference */
        real_t x_ref[MPC_HOR_STATES];  /* the state reference */
        geometry_msgs::PoseArray x_refPoseArray;
        x_refPoseArray.poses.empty();
        x_refPoseArray.header.frame_id = _currentPath.header.frame_id;
        x_refPoseArray.header.stamp = ros::Time::now();


        geometry_msgs::PoseArray x_predictPoseArray;
        x_predictPoseArray.header.frame_id = _currentPath.header.frame_id;
        x_predictPoseArray.header.stamp = ros::Time::now();

        // configure the mpc
        ctl.conf->in_iter = 15;
        ctl.conf->ex_iter = 10;
        ctl.conf->warmstart = true;


        std::cout << ctl.conf->ex_iter << std::endl;

        int i;
        /* set the input and state reference to the desired value */
        for (i=0; i<MPC_HOR_INPUTS; i++) {
            u_ref[i] = 0.;
        }




        geometry_msgs::PoseStamped currentStatePoseStamped;
        geometry_msgs::PoseStamped currentStatePoseStamped_pathframe;



        try{

            // if we tranfrom from world to map frame there is a quick way to do it
            if(currentState.frame_id == "/world" || currentState.frame_id == ""){
                currentStatePoseStamped_pathframe.header.frame_id = "/map";
                currentStatePoseStamped_pathframe.header.stamp = ros::Time::now();
                currentStatePoseStamped_pathframe.pose.position.x = currentState.position(0);
                currentStatePoseStamped_pathframe.pose.position.y = -currentState.position(1);
                currentStatePoseStamped_pathframe.pose.position.z = -currentState.position(2);
                currentStatePoseStamped_pathframe.pose.orientation =
                        tf::createQuaternionMsgFromRollPitchYaw(currentState.getEulerRPY()(0),
                                                                -currentState.getEulerRPY()(1),
                                                                -currentState.getEulerRPY()(2));
            }else{
                currentState.toROSGeometryMsgPoseStamped(currentStatePoseStamped);
                _tfListener.waitForTransform(_currentPath.header.frame_id,currentStatePoseStamped.header.frame_id,
                                             ros::Time::now(),ros::Duration(0.015));
                _tfListener.transformPose(_currentPath.header.frame_id,currentStatePoseStamped,
                                          currentStatePoseStamped_pathframe);
            }
            /* The current state */
            x[0] = currentStatePoseStamped_pathframe.pose.position.x;
            x[1] = currentStatePoseStamped_pathframe.pose.position.y;
            x[2] = currentStatePoseStamped_pathframe.pose.position.z;
            x[3] = tf::getYaw(currentStatePoseStamped_pathframe.pose.orientation);



            double dtt = 10.;
            int P = _currentPath.poses.size()-1;
            dtt = sqrt(pow(x[0] -_currentPath.poses[P].pose.position.x,2)
                    +pow(x[1] -_currentPath.poses[P].pose.position.y,2)
                    +pow(x[2] -_currentPath.poses[P].pose.position.z,2));

            //            if(dtt < 0.15){
            //                // pause everything hard for easier debugging
            //                ROS_WARN_STREAM("sleeping");
            //                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            //            }

            clock_t x_ref_begin = clock();
            this->buildXref(x,x_ref,_currentPath);
            clock_t x_ref_end = clock();
            std::cout << "builing x_ref took " << double(x_ref_end - x_ref_begin) << " ticks"
                      << " and " << double(x_ref_end-x_ref_begin)/CLOCKS_PER_SEC*1000 << " ms" << std::endl;

            /* assign the references to the mpc_ctl respective pointers */
            ctl.u_ref = u_ref;
            ctl.x_ref = x_ref;

            ROS_DEBUG_STREAM("distance remaining " << dtt);
            clock_t mpc_begin = clock();
            /* Solve MPC problem and print the input sequence */
            mpc_ctl_solve_problem(&ctl, x);
            clock_t mpc_end = clock();
            std::cout << "solving mpc tool " << double(mpc_end - mpc_begin) << " ticks"
                      << " and " << double(mpc_end-mpc_begin)/CLOCKS_PER_SEC*1000 << " ms" << std::endl;
            mpc_predict_next_state(&ctl, x);
            for (i=0; i < MPC_INPUTS; i++) {
                printf("u[%d] = %f ", i,(ctl.u_opt[i]));
            }
            std::cout << std::endl;

            geometry_msgs::Pose x_ref_pose;
            for(i=0;i< MPC_HOR; i++){
                x_ref_pose.position.x = ctl.x_ref[i*MPC_STATES+0];
                x_ref_pose.position.y = ctl.x_ref[i*MPC_STATES+1];
                x_ref_pose.position.z = ctl.x_ref[i*MPC_STATES+2];
                x_ref_pose.orientation = tf::createQuaternionMsgFromYaw(ctl.x_ref[i*MPC_STATES+3]);
                x_refPoseArray.poses.push_back(x_ref_pose);
            }

            _x_refPub.publish(x_refPoseArray);
            std::cout << "next state " << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << std::endl;
            _x_predict.position.x = x[0];
            _x_predict.position.y = x[1];
            _x_predict.position.z = x[2];
            _x_predict.orientation = tf::createQuaternionMsgFromYaw(x[3]);
            x_predictPoseArray.poses.push_back(_x_predict);

            _x_predictPub.publish(x_predictPoseArray);
            geometry_msgs::PoseStamped u_pathFrame;

            u_pathFrame.header.stamp = ros::Time::now();
            u_pathFrame.header.frame_id = _currentPath.header.frame_id;
            //            //  u_pathFrame.point = _x_predict;

            u_pathFrame.pose.position.x = ctl.u_opt[0];
            u_pathFrame.pose.position.y = ctl.u_opt[1];
            u_pathFrame.pose.position.z = ctl.u_opt[2];
            u_pathFrame.pose.orientation = tf::createQuaternionMsgFromYaw(ctl.u_opt[3]);


            geometry_msgs::PoseStamped u;

            // if we tranfrom from map to world frame there is a quick way to do it
            if(generatedTrajInput.frame_id == "/world" || generatedTrajInput.frame_id == ""){
                u.header.frame_id = "/map";
                u.header.stamp = ros::Time::now();
                u.pose.position.x = u_pathFrame.pose.position.x;
                u.pose.position.y = -u_pathFrame.pose.position.y;
                u.pose.position.z = -u_pathFrame.pose.position.z;
                u.pose.orientation =
                        tf::createQuaternionMsgFromYaw(-tf::getYaw(u_pathFrame.pose.orientation));
            }else{
                try{
                    _tfListener.waitForTransform(generatedTrajInput.frame_id,_currentPath.header.frame_id,
                                                 ros::Time::now(),ros::Duration(0.015));

                    _tfListener.transformPose(generatedTrajInput.frame_id,u_pathFrame,u);

                }catch(const tf::TransformException& ex){
                    ROS_ERROR("%s",ex.what());
                }
            }
            ROS_DEBUG_STREAM("u opt in world frame " << u.pose.position.x << " " << u.pose.position.y
                             << " " << u.pose.position.z);

            ROS_DEBUG_STREAM("x predict in world frame " << currentState.position(0) + u.pose.position.x
                             << " " << currentState.position(1) + u.pose.position.y
                             << " " <<  currentState.position(2) + u.pose.position.z);
            generatedTrajInput.setPosition(Vector3D(currentState.position(0) + u.pose.position.x,
                                                    currentState.position(1) + u.pose.position.y,
                                                    currentState.position(2) + u.pose.position.z));

            generatedTrajInput.setYawAngle(currentState.getEulerRPY()(2)+tf::getYaw(u.pose.orientation));

        }catch(...){
            ROS_ERROR_STREAM("no mpc today");
        }

        clock_t total_end = clock();
        std::cout << "the complete loop trajectory step active took " << double(total_end - total_begin) << " ticks"
                  << " or " << double(total_end - total_begin)/CLOCKS_PER_SEC*1000 << " ms" << std::endl;


    } // end is path called back


}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void mpcPathFollow::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    // Same as active
    generatedTrajInput.setHeader(ros::Time::now(),"");
    trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool mpcPathFollow::isValid(const TKState& currentState) const
{
    // never turns invalid
    return true;
}



}
