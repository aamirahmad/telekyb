
#include <telekyb_msgs/TKState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>

#ifndef KALMANDATATYPES_HPP_
#define KALMANDATATYPES_HPP_

// Buffers
struct StateBufferElement
{
	double timeStamp;
	//Eigen::Matrix<double,10,1> state;
    telekyb_msgs::TKState state;
	Eigen::Matrix<double,9,9> covariance;
};

struct MeasureBufferElement
{
		double timeStamp;
	geometry_msgs::TransformStamped measure;
	//int	sensorID;
	//Eigen::Matrix<double,6,9> covariance;
};

struct InputBufferElement
{
	double timeStamp;
	//Eigen::Matrix<double,6,1> input;
	sensor_msgs::Imu input;
};

#endif /* KALMANDATATYPES_HPP_ */
