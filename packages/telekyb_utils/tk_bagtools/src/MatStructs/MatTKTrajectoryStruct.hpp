#ifndef MATTKTRAJECTORYSTRUCT_HPP_
#define MATTKTRAJECTORYSTRUCT_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatTKTrajectoryStruct : public MatStruct {
public:
	MatTKTrajectoryStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & msgInst);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *position;
	mxArray *velocity;
	mxArray *acceleration;
	mxArray *jerk;
	mxArray *snap;
	mxArray *positionCtrlMode;
	mxArray *yawAngle;
	mxArray *yawRate;
	mxArray *yawAcceleration;
	mxArray *yawCtrlMode;
};


#endif /*MATTKTRAJECTORYSTRUCT_HPP_*/
