

#include "MatMassStampedStruct.hpp"
#include <mat.h>
#include <tk_draft_msgs/MassStamped.h>

MatMassStampedStruct::MatMassStampedStruct(std::string name_, unsigned long size) :
	MatStruct(name_, "tk_draft_msgs/MassStamped"),
	timeStamp(mxCreateDoubleMatrix(1,size,mxREAL)),
	mass(mxCreateDoubleMatrix(1,size,mxREAL)){
}

void MatMassStampedStruct::insert(unsigned long index, rosbag::MessageInstance const & msgInst){

	if (index >= mxGetN(timeStamp)){
		ROS_ERROR("Index out of size");
		return;
	}

	std::string dataType = msgInst.getDataType();
	if (dataType.compare(dataType) != 0) {
		ROS_ERROR("Wrong message type: received %s while expecting %s",dataType.c_str(),dataType.c_str());
		return;
	}

	tk_draft_msgs::MassStamped::ConstPtr msg = msgInst.instantiate<tk_draft_msgs::MassStamped>();

	double* timeStampPointer = mxGetPr(timeStamp) + index*mxGetM(timeStamp);
	double* massPointer = mxGetPr(mass) + index*mxGetM(mass);

	*(timeStampPointer) = msgInst.getTime().toSec();

	*(massPointer++) = msg->mass;
}

matError MatMassStampedStruct::toMatFile(MATFile *pmat){

	const char *field_names[] = {"timeStamp", "mass"};
	mwSize dims[2] = {1, 1};
	mxArray* matStruct = mxCreateStructArray(1, dims, sizeof(field_names)/sizeof(*field_names), field_names);

	int fieldIndex(0);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, timeStamp);
	mxSetFieldByNumber(matStruct, 0, fieldIndex++, mass);

	return matPutVariable(pmat, name.c_str(), matStruct);
};
