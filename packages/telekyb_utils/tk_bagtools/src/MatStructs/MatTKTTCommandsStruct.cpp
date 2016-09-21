

#include "MatTKTTCommandsStruct.hpp"
#include <mat.h>
#include <telekyb_msgs/TKTTCommands.h>

MatTKTTCommandsStruct::MatTKTTCommandsStruct(std::string name_, unsigned long size) :
	MatStruct(name_,"telekyb_msgs/TKTTCommands"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	thrust(mxCreateDoubleMatrix(1,size,mxREAL)),
	torque(mxCreateDoubleMatrix(3,size,mxREAL)){
}

void MatTKTTCommandsStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare(dataType) != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),dataType.c_str());
		return;
	}

	telekyb_msgs::TKTTCommands::ConstPtr msg = msgInst.instantiate<telekyb_msgs::TKTTCommands>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);;
	double* thrustPointer = mxGetPr(thrust) + index*mxGetM(thrust);
	double* torquePointer = mxGetPr(torque) + index*mxGetM(torque);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(thrustPointer++) = msg->thrust;

	*(torquePointer++) = msg->roll_torque;
	*(torquePointer++) = msg->pitch_torque;
	*(torquePointer) = msg->yaw_torque;

}

matError MatTKTTCommandsStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp","thrust","torque"};
	mwSize dims[2] = {1, 1};
	mxArray* stateStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, thrust);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, torque);

	return matPutVariable(pmat, name.c_str(), stateStruct);
};
