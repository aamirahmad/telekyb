#ifndef MATTKMOTORCOMMANDS_HPP_
#define MATTKMOTORCOMMANDS_HPP_

#include "MatStruct.hpp"
#include <mat.h>

class MatTKMotorCommandsStruct : public MatStruct {
public:
	MatTKMotorCommandsStruct(std::string name_, unsigned long size=0);
	void insert(unsigned long index, rosbag::MessageInstance const & msgInst);
	matError toMatFile(MATFile *pmat);

	mxArray *timeStamp;
	mxArray *commands;
};


#endif /*MATTKMOTORCOMMANDS_HPP_*/
