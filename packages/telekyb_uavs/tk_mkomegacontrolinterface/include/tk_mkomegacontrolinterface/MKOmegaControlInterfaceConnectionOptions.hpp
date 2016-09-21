/*
 * MKOmegaControlInterfaceConnectionOptions.hpp
 *
 *  Created on: Nov 23, 2011
 *  Updated: Dec, 2015
 *      Author: mriedel
 *      Coauthors: pstegagno, byueksel
 */

#ifndef MKOMEGACONTROLINTERFACECONNECTIONOPTIONS_HPP_
#define MKOMEGACONTROLINTERFACECONNECTIONOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/enum.hpp>
#include <telekyb_base/Options.hpp>

#include <termios.h>

#include <tk_mkinterface/MKInterfaceConnectionOptions.hpp>


namespace TELEKYB_NAMESPACE {


// For BaudRate
// TELEKYB_ENUM_VALUES(BaudRate, size_t,
// 	(BAUD4800)(B4800)
// 	(BAUD9600)(B9600)
// 	(BAUD19200)(B19200)
// 	(BAUD38400)(B38400)
// 	(BAUD57600)(B57600)
// 	(BAUD115200)(B115200)
// 	(BAUD230400)(B230400)
// )

class MKOmegaControlInterfaceConnectionOptions : public OptionContainer {
public:
//   Option< BaudRateBaseEnum<size_t>::Type >* tBaudRate;

  Option< int >* tImuDataPeriodus;

  Option< int >* tMotorDataPeriodus;

  Option< int >* tBatteryDataPeriodus;
  
  Option< std::string  >* tBaudRate;
  
  Option< std::string  >*  tImuStatePublisher;

  MKOmegaControlInterfaceConnectionOptions();
};

} /* namespace telekyb */
#endif /* MKOMEGACONTROLINTERFACECONNECTIONOPTIONS_HPP_ */
