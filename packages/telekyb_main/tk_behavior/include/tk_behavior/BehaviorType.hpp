/*
 * BehaviorTypes.hpp
 *
 *  Created on: Nov 17, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORTYPE_HPP_
#define BEHAVIORTYPE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>


namespace TELEKYB_NAMESPACE {

// define enum
TELEKYB_ENUM_VALUES(BehaviorType, const char*,
		(Ground)("Represents a behavior of Type Ground.")
		(TakeOff)("Represents a behavior that transitions form Ground to Air")
		(Land)("Represents a behavior that transitions form Air to Ground")
		(Air)("Represents a flying/air behavior")
		(Undef)("Do not use this type. It's never an allowed Transition")
)


}


#endif /* BEHAVIORTYPE_HPP_ */
