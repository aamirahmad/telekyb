#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include "telekyb_msgs/TKState.h"
#include "telekyb_srvs/BinOccupancySrv.h"

#define COEF 0.5
#define YAWCOEF 1.0

bool saveFirstState = true;
gazebo_msgs::ModelState firstState;
bool resetState = false;

gazebo_msgs::ModelState setModelState;
gazebo_msgs::GetModelState getModelState;
telekyb_srvs::BinOccupancySrv avoidObstacles;

ros::Publisher* gazeboPub;
ros::Publisher* tkStatePub;
ros::ServiceClient* gazeboClient;
ros::ServiceClient* obstacleClient;

Eigen::Vector3d NWU45LinVel;
Eigen::Vector3d NWU45AngVel;

Eigen::Matrix3d Ryaw45, RNEDNWU;
Eigen::Matrix3d RNWU45NED; 

telekyb_msgs::TKState tkStateMsg;

double R, P, Y;

double timerDuration;

void joyCB(const sensor_msgs::Joy::ConstPtr& msg)
{ 
  if (not(msg->buttons[8])) {
    NWU45LinVel(0) = COEF * msg->axes[1];
    NWU45LinVel(1) = COEF * msg->axes[0];
    NWU45LinVel(2) = COEF * msg->axes[4];
    NWU45AngVel(2) = YAWCOEF * msg->axes[3];
  } else {
    resetState = true;
  }
}

void stateTimerCB(const ros::TimerEvent&)
{
  gazeboClient->call(getModelState);
  
  if (getModelState.response.success) {
    if (saveFirstState) {
      firstState.model_name = getModelState.request.model_name;
      firstState.pose = getModelState.response.pose;
      firstState.twist = getModelState.response.twist;
      saveFirstState = false;
    }
    
    double roll, pitch, yaw;
    tf::Quaternion QqTF;
    Eigen::Matrix3d Rq; 
    
    tf::quaternionMsgToTF(getModelState.response.pose.orientation, QqTF);
    tf::Matrix3x3 RqTF(QqTF);
    
    RqTF.getRPY(roll, pitch, yaw);
    RqTF.setRPY(0.0, 0.0, yaw);
     //RqTF.setRPY(0.0, 0.0, yaw-0.78539);
    tf::matrixTFToEigen(RqTF, Rq);
    
    //Eigen::Vector3d NEDVel = RNEDNWU.transpose() * Ryaw45.transpose() * inputLinVel;
    Eigen::Vector3d NEDLinVelIn = RNWU45NED * NWU45LinVel;
    Eigen::Vector3d NEDLinVelOut;
    
    avoidObstacles.request.linear_velocity.x = NEDLinVelIn(0);
    avoidObstacles.request.linear_velocity.y = NEDLinVelIn(1);
    avoidObstacles.request.linear_velocity.z = NEDLinVelIn(2);
    
    if (fabs(NEDLinVelIn(0)*NEDLinVelIn(0) + NEDLinVelIn(1)*NEDLinVelIn(1)) > 1e-8 and obstacleClient->exists()) {
      obstacleClient->call(avoidObstacles);
    
      NEDLinVelOut(0) = avoidObstacles.response.linear_velocity.x;
      NEDLinVelOut(1) = avoidObstacles.response.linear_velocity.y;
      NEDLinVelOut(2) = avoidObstacles.response.linear_velocity.z;
    } else {
      NEDLinVelOut = NEDLinVelIn;
    }
    
    //World horizontal...
    Eigen::Vector3d WHLinVel = Rq * RNEDNWU * NEDLinVelOut;
    
    setModelState.model_name = getModelState.request.model_name;
    setModelState.pose = getModelState.response.pose;
    setModelState.twist.linear.x = WHLinVel(0);
    setModelState.twist.linear.y = WHLinVel(1);
    setModelState.twist.linear.z = WHLinVel(2);
    setModelState.twist.angular.z = NWU45AngVel(2);
    
    if (resetState) {
      setModelState = firstState;
      resetState = false;
    }
    
    gazeboPub->publish(setModelState);
    
    
    
    //-------------------------------- TK State --------------------------------//
    Eigen::Vector3d stateWLinVel, stateNEDLinVel;
    stateWLinVel(0) = getModelState.response.twist.linear.x;
    stateWLinVel(1) = getModelState.response.twist.linear.y;
    stateWLinVel(2) = getModelState.response.twist.linear.z;
    
    stateNEDLinVel = RNEDNWU * Rq.transpose() * stateWLinVel;
    
  // if (getModelState.response.success) {
      tkStateMsg.header.seq++;
      tkStateMsg.header.stamp = ros::Time::now();
    //  tkStateMsg.twist = getModelState.response.twist;
      tkStateMsg.pose = getModelState.response.pose;
      tkStateMsg.twist.linear.x = stateNEDLinVel(0);
      tkStateMsg.twist.linear.y = stateNEDLinVel(1);
      tkStateMsg.twist.linear.z = stateNEDLinVel(2);
      tkStateMsg.twist.angular.z = -(Y - yaw)/timerDuration;//"-": NWU -> NED
      tkStatePub->publish(tkStateMsg);
      
      Y = yaw;
    //} else {
    //  ROS_WARN("Cannot publish qc13 state\n");
    //}
  } else {
    ROS_WARN("Gazebo Model State Service Not Available");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazeboInterface");
  ros::NodeHandle n;
  
  timerDuration = 0.01;
  
  getModelState.request.model_name = "qc13";
  getModelState.request.relative_entity_name = "world";
  
  Ryaw45 << 	0.707, -0.707, 0.0,
		0.707, 0.707, 0.0,
		0.0, 0.0, 1.0;
		
  RNEDNWU <<	1.0,  0.0,  0.0,
		0.0, -1.0,  0.0,
		0.0,  0.0, -1.0;
		
  RNWU45NED = RNEDNWU.transpose() * Ryaw45.transpose();
  
  R = 0.0;
  P = 0.0;
  Y = 0.0;
  
  ros::Subscriber joySub = n.subscribe("/TeleKyb/tJoy/joy", 1000, joyCB);
  gazeboPub = new ros::Publisher(n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10));
  tkStatePub = new ros::Publisher(n.advertise<telekyb_msgs::TKState>("/TeleKyb/TeleKybCore_13/Sensor/TKState",10));
  gazeboClient = new ros::ServiceClient(n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"));
  obstacleClient = new ros::ServiceClient(n.serviceClient<telekyb_srvs::BinOccupancySrv>("/TeleKyb/BinOccupancyObsAvoid"));
  
  ros::Timer stateTimer = n.createTimer(ros::Duration(timerDuration), stateTimerCB); 
  
  ros::spin();

  return 0;
}
