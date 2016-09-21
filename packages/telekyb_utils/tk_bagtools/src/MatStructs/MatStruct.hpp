#ifndef MATSTRUCT_HPP_
#define MATSTRUCT_HPP_

#include <rosbag/bag.h>
#include <ros/ros.h>
#include <mat.h>

class MatStruct {
public:
	virtual void insert(unsigned long index, rosbag::MessageInstance const & msg) = 0;
	virtual matError toMatFile(MATFile *pmat) = 0;

	virtual void push_back(rosbag::MessageInstance const & msg);
	virtual unsigned long getCurrentSize();
	virtual ~MatStruct();

	MatStruct(std::string name_, std::string type_);

	std::string name;
	const std::string dataType;

protected:
	unsigned long currentSize;

};

#endif /*MATSTRUCT_HPP_*/
