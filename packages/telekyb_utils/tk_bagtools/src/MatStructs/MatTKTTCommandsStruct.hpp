#ifndef MATTKTTCOMMANDSSTRUCT_HPP_
#define MATTKTTCOMMANDSSTRUCT_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatTKTTCommandsStruct : public MatStruct {
public:
	MatTKTTCommandsStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & msgInst);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *thrust;
	mxArray *torque;
};


#endif /*MATTKTTCOMMANDSSTRUCT_HPP_*/
