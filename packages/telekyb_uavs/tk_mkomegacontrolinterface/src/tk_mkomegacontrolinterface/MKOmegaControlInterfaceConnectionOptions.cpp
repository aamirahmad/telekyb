/*
 * MKOmegaControlInterfaceConnectionOptions.cpp
 *
 *  Created on: Nov 23, 2011
 *  Updated: Dec, 2015
 *      Author: mriedel
 *      Coauthors: pstegagno, byueksel
 */

#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceConnectionOptions.hpp>

namespace TELEKYB_NAMESPACE {

MKOmegaControlInterfaceConnectionOptions::MKOmegaControlInterfaceConnectionOptions()
	: OptionContainer("MKOmegaControlInterfaceConnection")
{
  tBaudRate = addOption< std::string  >("tBaudRate", "Defines the BaudRate of the Serial Connection", "B230400", false, true);
//   tBaudRate = addOption< BaudRateBaseEnum<size_t>::Type >("tBaudRate", "Defines the BaudRate of the Serial Connection", BaudRate::BAUD230400, false, true);
  tImuDataPeriodus = addOption< int >("tImuDataPeriodus", "The period between two imu data packets in us", 1000 , false, false);
  tMotorDataPeriodus = addOption< int >("tMotorDataPeriodus", "The period between two motor data packets in us", 10000 , false, false);
  tBatteryDataPeriodus = addOption< int >("tBatteryDataPeriodus", "The period between two battery level data packets in us", 1000000 , false, false);
  tImuStatePublisher = addOption<std::string>("tImuStatePublisher","Topic for publishing IMU data","imu", false, true);
}

} /* namespace telekyb */
