

#include "MatPoseStampedStruct.hpp"
#include <mat.h>
#include <geometry_msgs/PoseStamped.h>

MatPoseStampedStruct::MatPoseStampedStruct(std::string name_, unsigned long size) :
	MatStruct(name_, "geometry_msgs/PoseStamped"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	position(mxCreateDoubleMatrix(3,size,mxREAL)),
	orientation(mxCreateDoubleMatrix(4,size,mxREAL)){
}

void MatPoseStampedStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare("geometry_msgs/PoseStamped") != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),"geometry_msgs/PoseStamped");
		return;
	}

	geometry_msgs::PoseStamped::ConstPtr msg = msgInst.instantiate<geometry_msgs::PoseStamped>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);
	double* positionPointer = mxGetPr(position) + index*mxGetM(position);
	double* orientationPointer = mxGetPr(orientation) + index*mxGetM(orientation);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(positionPointer++) = msg->pose.position.x;
	*(positionPointer++) = msg->pose.position.y;
	*(positionPointer) = msg->pose.position.z;

	*(orientationPointer++) = msg->pose.orientation.w;
	*(orientationPointer++) = msg->pose.orientation.x;
	*(orientationPointer++) = msg->pose.orientation.y;
	*(orientationPointer) = msg->pose.orientation.z;

}

matError MatPoseStampedStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp","position","orientation"};
	mwSize dims[2] = {1, 1};
	mxArray* stateStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, position);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, orientation);

	return matPutVariable(pmat, name.c_str(), stateStruct);
};
