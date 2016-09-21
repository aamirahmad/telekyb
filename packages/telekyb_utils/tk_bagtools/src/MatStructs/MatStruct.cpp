#include "MatStruct.hpp"

void MatStruct::push_back(rosbag::MessageInstance const & msgInst){
	insert(currentSize++, msgInst);
}

unsigned long MatStruct::getCurrentSize(){
	return currentSize;
}

MatStruct::MatStruct(std::string name_, std::string type_):
		name(name_),
		dataType(type_),
		currentSize(0){
};

MatStruct::~MatStruct(){};
