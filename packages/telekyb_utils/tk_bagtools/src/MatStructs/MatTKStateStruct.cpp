

#include "MatTKStateStruct.hpp"
#include <mat.h>
#include <telekyb_msgs/TKState.h>

MatTKStateStruct::MatTKStateStruct(std::string name_, unsigned long size) :
	MatStruct(name_,"telekyb_msgs/TKState"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	position(mxCreateDoubleMatrix(3,size,mxREAL)),
	linVelocity(mxCreateDoubleMatrix(3,size,mxREAL)),
	orientation(mxCreateDoubleMatrix(4,size,mxREAL)),
	angVelocity(mxCreateDoubleMatrix(3,size,mxREAL)){
}

void MatTKStateStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare(dataType) != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),dataType.c_str());
		return;
	}

	telekyb_msgs::TKState::ConstPtr msg = msgInst.instantiate<telekyb_msgs::TKState>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);
	double* positionPointer = mxGetPr(position) + index*mxGetM(position);
	double* linVelocityPointer = mxGetPr(linVelocity) + index*mxGetM(linVelocity);
	double* orientationPointer = mxGetPr(orientation) + index*mxGetM(orientation);
	double* angVelocityPointer = mxGetPr(angVelocity) + index*mxGetM(angVelocity);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(positionPointer++) = msg->pose.position.x;
	*(positionPointer++) = msg->pose.position.y;
	*(positionPointer) = msg->pose.position.z;

	*(linVelocityPointer++) = msg->twist.linear.x;
	*(linVelocityPointer++) = msg->twist.linear.y;
	*(linVelocityPointer) = msg->twist.linear.z;

	*(orientationPointer++) = msg->pose.orientation.w;
	*(orientationPointer++) = msg->pose.orientation.x;
	*(orientationPointer++) = msg->pose.orientation.y;
	*(orientationPointer) = msg->pose.orientation.z;

	*(angVelocityPointer++) = msg->twist.angular.x;
	*(angVelocityPointer++) = msg->twist.angular.y;
	*(angVelocityPointer) = msg->twist.angular.z;

}

matError MatTKStateStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp","position","linVelocity","orientation","angVelocity"};
	mwSize dims[2] = {1, 1};
	mxArray* stateStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, position);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, linVelocity);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, orientation);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, angVelocity);

	return matPutVariable(pmat, name.c_str(), stateStruct);
};
