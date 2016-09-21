/**
 * main.cpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Oct 2, 2014
 *      Author: Johannes Lächele
 *  
 */

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Base.hpp>
#include <telekyb_base/Options.hpp>

#include "tkwittenstein.hpp"

using namespace telekyb;

int main(int argc, char* argv[]) {
	// only outputs received wittenstein values, no spinner thread needed
	RawOptionsContainer::addOption("tRosNrSpinnerThreads","-1");
	TeleKyb::init(argc,argv,"tJoy", ros::init_options::AnonymousName);

	tk_wittenstein wittenstein;
	wittenstein.run();
//	Joystick j(options->tDevicePath->getValue(), true);
//	j.run();
//
//	j.closeJoystick();

	// it is important to free everything before this call.

	TeleKyb::shutdown();
	return 0;
}

