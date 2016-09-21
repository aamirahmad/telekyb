

#include "MatTKMotorCommandsStruct.hpp"
#include <mat.h>
#include <telekyb_msgs/TKMotorCommands.h>

MatTKMotorCommandsStruct::MatTKMotorCommandsStruct(std::string name_, unsigned long size) :
	MatStruct(name_,"telekyb_msgs/TKMotorCommands"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	commands(mxCreateDoubleMatrix(4,size,mxREAL)){
}

void MatTKMotorCommandsStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare(dataType) != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),dataType.c_str());
		return;
	}

	telekyb_msgs::TKMotorCommands::ConstPtr msg = msgInst.instantiate<telekyb_msgs::TKMotorCommands>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);;
	double* commandsPointer = mxGetPr(commands) + index*mxGetM(commands);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(commandsPointer++) = msg->force[0];
	*(commandsPointer++) = msg->force[1];
	*(commandsPointer++) = msg->force[2];
	*(commandsPointer) = msg->force[3];

}

matError MatTKMotorCommandsStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp","commands"};
	mwSize dims[2] = {1, 1};
	mxArray* stateStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(stateStruct, 0, fieldIndex++, commands);

	return matPutVariable(pmat, name.c_str(), stateStruct);
};
