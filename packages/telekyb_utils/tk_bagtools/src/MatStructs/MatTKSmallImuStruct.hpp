#ifndef MATTKSMALLIMUSTRUCT_HPP_
#define MATTKSMALLIMUSTRUCT_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatTKSmallImuStruct : public MatStruct {
public:
	MatTKSmallImuStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & msgInst);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *linAcceleration;
	mxArray *angVelocity;
};


#endif /*MATTKSMALLIMUSTRUCT_HPP_*/
