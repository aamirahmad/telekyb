

#include "MatTKTrajectoryStruct.hpp"
#include <mat.h>
#include <telekyb_msgs/TKTrajectory.h>

MatTKTrajectoryStruct::MatTKTrajectoryStruct(std::string name_, unsigned long size) :
	MatStruct(name_, "telekyb_msgs/TKTrajectory"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	position(mxCreateDoubleMatrix(3,size,mxREAL)),
	velocity(mxCreateDoubleMatrix(3,size,mxREAL)),
	acceleration(mxCreateDoubleMatrix(3,size,mxREAL)),
	jerk(mxCreateDoubleMatrix(3,size,mxREAL)),
	snap(mxCreateDoubleMatrix(3,size,mxREAL)),
	positionCtrlMode(mxCreateNumericMatrix(3,size,mxUINT8_CLASS,mxREAL)),
	yawAngle(mxCreateDoubleMatrix(1,size,mxREAL)),
	yawRate(mxCreateDoubleMatrix(1,size,mxREAL)),
	yawAcceleration(mxCreateDoubleMatrix(1,size,mxREAL)),
	yawCtrlMode(mxCreateNumericMatrix(1,size,mxUINT8_CLASS,mxREAL)){
}

void MatTKTrajectoryStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare(dataType) != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),dataType.c_str());
		return;
	}

	telekyb_msgs::TKTrajectory::ConstPtr msg = msgInst.instantiate<telekyb_msgs::TKTrajectory>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);
	double* positionPointer = mxGetPr(position) + index*mxGetM(position);
	double* velocityPointer = mxGetPr(velocity) + index*mxGetM(velocity);
	double* accelerationPointer = mxGetPr(acceleration) + index*mxGetM(acceleration);
	double* jerkPointer = mxGetPr(jerk) + index*mxGetM(jerk);
	double* snapPointer = mxGetPr(snap) + index*mxGetM(snap);
	uint8_T* positionCtrlModePointer = (uint8_T*)mxGetPr(positionCtrlMode) + index*mxGetM(positionCtrlMode);
	double* yawAnglePointer = mxGetPr(yawAngle) + index*mxGetM(yawAngle);
	double* yawRatePointer = mxGetPr(yawRate) + index*mxGetM(yawRate);
	double* yawAccelerationPointer = mxGetPr(yawAcceleration) + index*mxGetM(yawAcceleration);
	uint8_T* yawCtrlModePointer = (uint8_T*)mxGetPr(yawCtrlMode) + index*mxGetM(yawCtrlMode);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(positionPointer++) = msg->position.x;
	*(positionPointer++) = msg->position.y;
	*(positionPointer) = msg->position.z;

	*(velocityPointer++) = msg->velocity.x;
	*(velocityPointer++) = msg->velocity.y;
	*(velocityPointer) = msg->velocity.z;

	*(accelerationPointer++) = msg->acceleration.x;
	*(accelerationPointer++) = msg->acceleration.y;
	*(accelerationPointer) = msg->acceleration.z;

	*(jerkPointer++) = msg->jerk.x;
	*(jerkPointer++) = msg->jerk.y;
	*(jerkPointer) = msg->jerk.z;

	*(snapPointer++) = msg->snap.x;
	*(snapPointer++) = msg->snap.y;
	*(snapPointer)= msg->snap.z;

	*(positionCtrlModePointer++) = msg->xAxisCtrlType;
	*(positionCtrlModePointer++) = msg->yAxisCtrlType;
	*(positionCtrlModePointer) = msg->zAxisCtrlType;

	*(yawAnglePointer) = msg->yawAngle;
	*(yawRatePointer) = msg->yawRate;
	*(yawAccelerationPointer) = msg->yawAcceleration;
	*(yawCtrlModePointer) = msg->yawCtrlType;

}

matError MatTKTrajectoryStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp", "position",  "velocity", "acceleration", "jerk", "snap", "positionCtrlMode", "yawAngle", "yawRate", "yawAcceleration", "yawCtrlMode"};
	mwSize dims[2] = {1, 1};
	mxArray* matStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, position);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, velocity);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, acceleration);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, jerk);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, snap);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, positionCtrlMode);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, yawAngle);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, yawRate);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, yawAcceleration);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, yawCtrlMode);

	return matPutVariable(pmat, name.c_str(), matStruct);
};
