#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>

#include <telekyb_msgs/TKCommands.h>
#include <telekyb_msgs/TKMotorCommands.h>
#include <telekyb_base/ROS.hpp>

ros::Publisher command_pub;
ros::Publisher motor_command_pub;
ros::Publisher vicon_pub;
ros::Timer vicon_timer;

geometry_msgs::PoseStamped vicmsg;
bool viconPublished;
int nmpcActive = 0;

void imuCallback(const sensor_msgs::ImuConstPtr& msg){
  // Handle IMU data.
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  // Handle pose measurements.
}

void poseGTCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  // Handle pose measurements.
  if (!viconPublished) {
    vicmsg.header = msg->header;
    vicmsg.pose = msg->pose.pose;
  }
}

void viconTimerCallback(const ros::TimerEvent& event)
{
    viconPublished = true;
    vicon_pub.publish(vicmsg);
    viconPublished = false;
}

void tk_commandsCallback(const telekyb_msgs::TKCommands::ConstPtr& msg){
  // Handle tk_commands: translate them in eurocmsgs and publish them.

    // Example for using the attitude controller:
    mav_msgs::CommandAttitudeThrust output_msg;
    output_msg.header = msg->header; // use the latest information you have.
    output_msg.roll = msg->roll; // roll angle [rad]
    output_msg.pitch = -msg->pitch; // pitch angle  [rad]
    output_msg.yaw_rate = -msg->yaw; // yaw rate around z-axis [rad/s]
    output_msg.thrust = -msg->thrust; // thrust [N]
    
    if(nmpcActive==0)
    {
      std::cout<<"we have no NMPC"<<std::endl;
      command_pub.publish(output_msg);
    }
}

void tk_motorCommandsCallback(const telekyb_msgs::TKMotorCommands::ConstPtr& msg){
  // Handle tk_commands: translate them in eurocmsgs and publish them.
    // Example for using the motor controller:
    mav_msgs::CommandMotorSpeed output_msg;
    output_msg.header = msg->header; // use the latest information you have.
    output_msg.motor_speed.push_back(msg->force[0]); // roll angle [rad]
    output_msg.motor_speed.push_back(msg->force[1]); // pitch angle  [rad]
    output_msg.motor_speed.push_back(msg->force[2]); // yaw rate around z-axis [rad/s]
    output_msg.motor_speed.push_back(msg->force[3]); // thrust [N]
    output_msg.motor_speed.push_back(msg->force[4]); // thrust [N]
    output_msg.motor_speed.push_back(msg->force[5]); // thrust [N]
    motor_command_pub.publish(output_msg);
}


void nmpcActivityCallback(const std_msgs::Int8::ConstPtr& msg)
{
  nmpcActive = msg->data;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "tk_gazebo_interface");

  ros::NodeHandle nh(telekyb::ROSModule::Instance().getMainNodeHandle());


  // get the paramter from the ros parameter server
  int robotID = 0;
  std::string robotNamespace("default");

//  nh.getParam("robotID",robotID);
//  nh.getParam("robotName",robotNamespace);
  nh.getParam("/euroc_solution/robotID",robotID);
  nh.getParam("/euroc_solution/robotName",robotNamespace);

std::cout << "robotID " << robotID << std::endl;
std::cout << "robotNamespace " << robotNamespace << std::endl;

  //ROS_DEBUG_STREAM("nh.getNamespace() " << nh.getNamespace());
  //ROS_DEBUG_STREAM("param robotNamespace " << robotNamespace);


  ROS_DEBUG("running tk_gazebo_interface");

  std::stringstream commandAttitudeTopic;
  commandAttitudeTopic << "/" << robotNamespace << "/command/attitude";
  ROS_DEBUG_STREAM("commandAttitudeTopic " << commandAttitudeTopic.str());
  // Chose one of the versions below. The first of these topic published determines the control mode.
  command_pub = nh.advertise<mav_msgs::CommandAttitudeThrust>(commandAttitudeTopic.str(), 10);
  //ros::Publisher command_pub = nh.advertise<mav_msgs::CommandRateThrust>("command/rate", 10);

  std::stringstream commandMotorsTopic;
  commandMotorsTopic  << "/" << robotNamespace << "/command/motors";
  ROS_DEBUG_STREAM("commandMotorsTopic " << commandMotorsTopic.str());
  motor_command_pub = nh.advertise<mav_msgs::CommandMotorSpeed>(commandMotorsTopic.str(), 10);


  std::stringstream viconTopic;
  viconTopic  << "/TeleKyb/Vicon/Quadcopter_" << robotID << "/Quadcopter_" << robotID;
  ROS_DEBUG_STREAM("viconTopic " << viconTopic.str());
  vicon_pub = nh.advertise<geometry_msgs::PoseStamped>(viconTopic.str(), 10);



  // pose ground truth
  std::stringstream poseGTTopic;
  poseGTTopic << "/" << robotNamespace << "/ground_truth/pose";
  ROS_DEBUG_STREAM("poseGTTopic topic " << poseGTTopic.str());
  ros::Subscriber poseGT_sub = nh.subscribe(poseGTTopic.str(), 1, &poseGTCallback);

  // sensors
  std::stringstream imuTopic;
  imuTopic << "/" << robotNamespace << "/imu";
  ROS_DEBUG_STREAM("imuTopic " << imuTopic.str());
  ros::Subscriber imu_sub = nh.subscribe(imuTopic.str(), 10, &imuCallback);

  std::stringstream poseTopic;
  poseTopic << "/" << robotNamespace << "/pose";
  ROS_DEBUG_STREAM("poseTopic " << poseTopic.str());
  ros::Subscriber pose_sub = nh.subscribe(poseTopic.str(), 10, &poseCallback);

  // topics from telekyb

  std::stringstream commandTopic;
  commandTopic << "/TeleKyb/" << robotID << "/Commands";
  ROS_DEBUG_STREAM("tk gazebo interface::main commandTopic " << commandTopic.str());
  ros::Subscriber tk_command = nh.subscribe(commandTopic.str(), 10, &tk_commandsCallback);

  std::stringstream motorCommandTopic;
  motorCommandTopic << "/TeleKyb/" << robotID << "/MotorCommands";
  ROS_DEBUG_STREAM("tk gazebo interface::main motorCommandTopic " << motorCommandTopic.str());
  ros::Subscriber tk_motor_command = nh.subscribe(motorCommandTopic.str(), 10, &tk_motorCommandsCallback);

  std::stringstream nmpcActivityTopic;
  nmpcActivityTopic << "/firefly_1/NMPCActive";
  ROS_DEBUG_STREAM("tk gazebo interface::main nmpcActivityTopic " << nmpcActivityTopic.str());
  ros::Subscriber sub_nmpc_activity = nh.subscribe(nmpcActivityTopic.str(), 1000, &nmpcActivityCallback);



  vicon_timer = nh.createTimer(ros::Duration(1.0/120.0), viconTimerCallback);
  viconPublished = false;

  // Wake up simulation, from this point on, you have 30s to initialize
  // everything and fly to the evaluation position (x=0m y=0m z=1m).
  ROS_INFO("Waking up simulation ... ");
  std_srvs::Empty srv;
  bool ret = ros::service::call("/gazebo/unpause_physics", srv);

  if(ret)
    ROS_INFO("... ok");
  else{
    ROS_FATAL("tk_gazebo_interface:: could not wake up gazebo");
    return -1;
  }

  // Initialize your filter / controller.

  // Run the control loop and Fly to x=0m y=0m z=1m.
  while(ros::ok()){
    ros::spinOnce();
    // Check if imu/pose data was received
    // Compute state estimate
    // Compute control outputs

    ros::Duration(0.001).sleep(); // may be set slower.
  }

  // 30s after waking up the simulation, we measure for 30s

  return 0;
}


