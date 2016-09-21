

#include "MatTKSmallImuStruct.hpp"
#include <mat.h>
#include <tk_draft_msgs/TKSmallImu.h>

MatTKSmallImuStruct::MatTKSmallImuStruct(std::string name_, unsigned long size) :
	MatStruct(name_,"tk_draft_msgs/TKSmallImu"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	linAcceleration(mxCreateDoubleMatrix(3,size,mxREAL)),
	angVelocity(mxCreateDoubleMatrix(3,size,mxREAL)){
}

void MatTKSmallImuStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare(dataType) != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),dataType.c_str());
		return;
	}

	tk_draft_msgs::TKSmallImu::ConstPtr msg = msgInst.instantiate<tk_draft_msgs::TKSmallImu>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);
	double* linAccelerationPointer = mxGetPr(linAcceleration) + index*mxGetM(linAcceleration);
	double* angVelocityPointer = mxGetPr(angVelocity) + index*mxGetM(angVelocity);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(linAccelerationPointer++) = msg->linear_acceleration.x;
	*(linAccelerationPointer++) = msg->linear_acceleration.y;
	*(linAccelerationPointer) = msg->linear_acceleration.z;

	*(angVelocityPointer++) = msg->angular_velocity.x;
	*(angVelocityPointer++) = msg->angular_velocity.y;
	*(angVelocityPointer) = msg->angular_velocity.z;

}

matError MatTKSmallImuStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp","linAcceleration","angVelocity"};
	mwSize dims[2] = {1, 1};
	mxArray* stateStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, linAcceleration);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, angVelocity);

	return matPutVariable(pmat, name.c_str(), stateStruct);
};
