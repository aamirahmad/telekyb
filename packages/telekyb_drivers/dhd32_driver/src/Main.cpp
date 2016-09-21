/*
 * Main.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <telekyb_base/TeleKyb.hpp>

#include <boost/foreach.hpp>

#include "DHDDriver.hpp"

using namespace TELEKYB_NAMESPACE;


int main(int argc, char **argv)
{
	TeleKyb::init(argc,argv,"dhd32_driver");

	DHDDriver* driver = new DHDDriver();
	driver->start(); // blocking
	delete driver;

	TeleKyb::shutdown();
}


