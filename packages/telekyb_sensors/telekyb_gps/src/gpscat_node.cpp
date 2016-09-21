#include <ros/ros.h>
#include <stdio.h>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include "opencv2/opencv.hpp"
#include <fstream>
#include <telekyb_base/Time.hpp>

using namespace std;
using namespace cv;

fstream logfile;

double EARTH_RADIUS = 6371000.0; // radius in m
double lat = 0, lon = 0, alt = 0;
uint seq = 0;
bool firstReading = false; // used to set the origin when performing the first reading

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	if (alt== 0 && lon == 0 && lat == 0) { // Look for another solution if possible
		firstReading = true;
	}
	seq = msg->header.seq;
	alt = (double)(msg->altitude);
	lat = (double)(msg->latitude / 180.0 * M_PI);
	lon = (double)(msg->longitude / 180.0 * M_PI);
	logfile.precision(30);
	logfile << lat << "  " << lon << "  "<< alt << endl;
	cout << "seq " << seq << "   " << lat << "  " << lon << "  "<< alt << endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpsDataLogger");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/fix", 1, gpsCallback);
	geometry_msgs::Point p;
	ros::Rate loop_rate(18);

	stringstream filename;
	TELEKYB_NAMESPACE::Time tim;
	filename << "gpsfile-" << tim.dateTimeToString()<< ".txt";
	cout << filename.str().c_str() << endl;

	logfile.open(filename.str().c_str(), std::fstream::out );

	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
