#ifndef MATTKSTATESTRUCT_HPP_
#define MATTKSTATESTRUCT_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatTKStateStruct : public MatStruct {
public:
	MatTKStateStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & msgInst);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *position;
	mxArray *linVelocity;
	mxArray *orientation;
	mxArray *angVelocity;
};


#endif /*MATTKSTATESTRUCT_HPP_*/
