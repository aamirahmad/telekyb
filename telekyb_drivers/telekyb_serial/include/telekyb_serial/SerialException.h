/*
 * SerialException.h
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#ifndef SERIALEXCEPTION_H_
#define SERIALEXCEPTION_H_

#include <telekyb_defines/telekyb_defines.hpp>
// enum
#include <telekyb_defines/telekyb_enums.hpp>

#include <exception>

// ros
#include <ros/console.h>

namespace TELEKYB_NAMESPACE
{

TELEKYB_ENUM(SerialExceptionCode,
		(NO_ERROR)
		(UNABLE_TO_OPEN)
		(LOCKED)
		(NO_TTY)
		(IO_ERROR)
		(GENERAL_ERROR))


class SerialException : public std::exception {
protected:
	std::string msg;
	ros::console::Level level;
	bool processed;

public:
	SerialException(const std::string& msg_,
			SerialExceptionCode code_ = SerialExceptionCode::GENERAL_ERROR,
			ros::console::Level level_ = ros::console::levels::Error);
	virtual ~SerialException() throw();

	// Prints ROS Message with Level. Fatal halts execution
	void process();

	// ErrorCode
	SerialExceptionCode code;
};

} // namespace

#endif /* SERIALEXCEPTION_H_ */
