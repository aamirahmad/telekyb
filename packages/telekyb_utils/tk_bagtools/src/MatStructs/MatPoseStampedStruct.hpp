#ifndef MATPOSESTAMPEDSTRUCT_HPP_
#define MATPOSESTAMPEDSTRUCT_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatPoseStampedStruct : public MatStruct {
public:
	MatPoseStampedStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & m);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *position;
	mxArray *orientation;

};


#endif /*MATPOSESTAMPEDSTRUCT_HPP_*/
