/*
 * BehaviorHelper.hpp
 *
 *  Created on: Nov 17, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORHELPER_HPP_
#define BEHAVIORHELPER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/BehaviorType.hpp>

#include <tk_behavior/Behavior.hpp>

namespace TELEKYB_NAMESPACE {

class BehaviorHelper {
public:
	static bool isAllowedTypeTransition(const BehaviorType& from, const BehaviorType& to);
	static bool isAllowedBehaviorTransition(const Behavior& from, const Behavior& to);

};

inline
bool BehaviorHelper::isAllowedTypeTransition(const BehaviorType& from, const BehaviorType& to) {

	return (
			(from == BehaviorType::Air && to == BehaviorType::Air) || 	// Air -> Air
			(from == BehaviorType::TakeOff && to == BehaviorType::Air) || // Takeoff -> Air
			(from == BehaviorType::Air && to == BehaviorType::Land) || // Air -> Land
			(from == BehaviorType::TakeOff && to == BehaviorType::Land) || // TakeOff->Land
			(from == BehaviorType::Ground && to == BehaviorType::TakeOff) || // Ground -> Takeoff
			(from == BehaviorType::Land && to == BehaviorType::Ground) || // Land -> Ground
			(from == BehaviorType::Ground && to == BehaviorType::Ground) // Ground -> Ground
			);
}

inline
bool BehaviorHelper::isAllowedBehaviorTransition(const Behavior& from, const Behavior& to) {
	return isAllowedTypeTransition(from.getType(), to.getType());
}

} // namespace

#endif /* BEHAVIORHELPER_HPP_ */
