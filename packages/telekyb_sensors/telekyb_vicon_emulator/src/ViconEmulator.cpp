#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

using namespace std;

int main(int argc, char **argv)
{
	if (argc<2){
		ROS_ERROR("ViconEmulator: Did not set mandatory option id. ViconEmulator is not able to work!");
		return 0;
	}
	
	stringstream topicName;
	topicName << "/TeleKyb/Vicon/Quadcopter_" << argv[1]  << "/Quadcopter_" << argv[1];
	ros::init(argc, argv, "viconEmulator");
	ros::NodeHandle n;
	ros::Publisher pathStatusPublisher = n.advertise<geometry_msgs::PoseStamped>(topicName.str().c_str(), 1);
	geometry_msgs::PoseStamped fakePose;
	
	ros::Time now;
	// fake header
	fakePose.header.seq=0;
	fakePose.header.stamp=now;
//	fakePose.header.stamp.nsecs=0;
	//fakePose.header.frame_id="Quadcopter_3/Quadcopter_3";

	// fake position
	fakePose.pose.position.x=0.0;
	fakePose.pose.position.y=0.0;
	fakePose.pose.position.z=0.0;

	// fake orientation
	fakePose.pose.orientation.x=0.0;
	fakePose.pose.orientation.y=0.0;
	fakePose.pose.orientation.z=1.0;
	fakePose.pose.orientation.w=0.0;

	ros::Rate loop_rate(100);
	while (ros::ok()) {
		fakePose.pose.position.z+=1;
		fakePose.header.seq++;
		pathStatusPublisher.publish(fakePose);
		loop_rate.sleep();
	}
	return 0;
}
