#include <ros/ros.h>
#include <stdio.h>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

double EARTH_RADIUS = 6371000.0; // radius in m
double lat = 0, lon = 0, alt = 0;
uint seq = 0, lastseq = 0;
bool firstReading = false; // used to set the origin when performing the first reading

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	if (alt== 0 && lon == 0 && lat == 0) { // Look for another solution if possible
		firstReading = true;
	}
	seq = msg->header.seq;
	alt = (double)(msg->altitude);
//	converting latitude and longtude from degrees to radians
	lat = (double)(msg->latitude / 180.0 * M_PI);
	lon = (double)(msg->longitude / 180.0 * M_PI);
//	cout << "alt: " << alt << " lat: " << lat << " lon:" << lon << endl;
}


Mat transformationToNEDframe()
{
	double rotationMatrixAroundZ[9] = {cos(lon), sin(lon), 0, -sin(lon), cos(lon), 0, 0, 0, 1.0};
	Mat RZ(3, 3, CV_64FC1, rotationMatrixAroundZ);
	double rotationMatrixAroundY[9] = {cos(-lat - M_PI/2.0), 0, -sin(-lat - M_PI/2.0), 0, 1.0, 0, sin(-lat - M_PI/2.0), 0, cos(-lat - M_PI/2.0)};
	Mat RY(3, 3, CV_64FC1, rotationMatrixAroundY);
	Mat Rtot = RY*RZ; //  total rotation of the coordinate frame
	return Rtot;
}

Mat transformationToENUframe()
{
	double rotationMatrixAroundZ1[9] = {cos(lon), sin(lon), 0, -sin(lon), cos(lon), 0, 0, 0, 1.0};
	Mat RZ1(3, 3, CV_64FC1, rotationMatrixAroundZ1);
	double rotationMatrixAroundY[9] = {cos(M_PI/2.0 - lat), 0, -sin(M_PI/2.0 - lat), 0, 1.0, 0, sin(M_PI/2.0 - lat), 0, cos(M_PI/2.0 - lat)};
	Mat RY(3, 3, CV_64FC1, rotationMatrixAroundY);
	double rotationMatrixAroundZ2[9] = {cos(M_PI/2.0), sin(M_PI/2.0), 0, -sin(M_PI/2.0), cos(M_PI/2.0), 0, 0, 0, 1.0};
	Mat RZ2(3, 3, CV_64FC1, rotationMatrixAroundZ2);
	Mat Rtot = RZ2*RY*RZ1; // total rotation of the coordinate frame
	/* Rotation matrices are transposed, because they are considered in the reference frame of the XYZPoint*/
	return Rtot;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "converter");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/fix", 1, gpsCallback);
	geometry_msgs::Point p;
	ros::Publisher xyzPublisher = n.advertise<geometry_msgs::Point>("point3D", 1);
	ros::Rate loop_rate(18);

//    Find a better way to do this stuff --------------------------
	Mat RotationMatrix = transformationToENUframe();
	double translationMatrixAlongZ[3] = {0, 0, 0};
	Mat translationVector(3, 1, CV_64FC1, translationMatrixAlongZ);
//    --------------------------------------------------------------
	while (ros::ok()) {
		if (firstReading) {
			RotationMatrix = transformationToENUframe();
			translationVector.at<double>(2,0) = EARTH_RADIUS + alt;
			firstReading = false;
		}
		double cartesianCoordinates[3] = {(EARTH_RADIUS + alt) * cos(lat)*cos(lon),
				(EARTH_RADIUS + alt) * cos(lat)*sin(lon),
				(EARTH_RADIUS + alt) * sin(lat)};

//		cout << "x: " << cartesianCoordinates[0] << " y: " << cartesianCoordinates[1] << " z:" << cartesianCoordinates[2] << endl;

//		double cartesianCoordinates[3] = {(EARTH_RADIUS) * cos(lat)*cos(lon) + alt * cos(lat)*cos(lon),
//						(EARTH_RADIUS) * cos(lat)*sin(lon) + alt * cos(lat)*sin(lon),
//						(EARTH_RADIUS) * sin(lat) + alt * sin(lat)};

		Mat point3D(3, 1, CV_64FC1, cartesianCoordinates);
		point3D = RotationMatrix * point3D - translationVector;

//		cout << "x1: " << point3D.at<double>(0,0) << " y1: " << point3D.at<double>(1,0) << " z1:" << point3D.at<double>(2,0) << endl;

//		p.x = cartesianCoordinates[0];
//		p.y = cartesianCoordinates[1];
//		p.z = cartesianCoordinates[2];

//		p.x = seq;
	 	p.x = point3D.at<double>(0,0);
		p.y = point3D.at<double>(1,0);
		p.z = point3D.at<double>(2,0);
		if (lastseq!=seq) {
			xyzPublisher.publish(p);
		}
		lastseq = seq;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
