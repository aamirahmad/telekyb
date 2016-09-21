#ifndef MATMASSSTAMPEDSTRUCT_HPP_
#define MATMASSSTAMPEDSTRUCT_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatMassStampedStruct : public MatStruct {
public:
	MatMassStampedStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & msgInst);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *mass;
};


#endif /*MATMASSSTAMPEDSTRUCT_HPP_*/
