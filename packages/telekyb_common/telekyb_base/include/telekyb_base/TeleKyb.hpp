/*
 * TeleKyb.hpp
 *
 *  Created on: Oct 17, 2011
 *      Author: mriedel
 */

#ifndef META_TELEKYB_HPP_
#define META_TELEKYB_HPP_

#include <string>
#include <ros/types.h>
#include <ros/spinner.h>

#include <telekyb_defines/telekyb_defines.hpp>

namespace TELEKYB_NAMESPACE
{

class TeleKyb {
private:
	static bool initialized;

public:
	static void init(int argc, char* argv[], const std::string& name, uint32_t options = 0);
	static bool isInitialized();

	// shutdown static Instances
	static void shutdown();
};

}

#endif /* TELEKYB_HPP_ */
