/*
 * ClassInterfaces.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef CLASSINTERFACES_HPP_
#define CLASSINTERFACES_HPP_

class BehaviorSwitcher {
public:
	//virtual ~BehaviorSwitcher() {};
	virtual bool switchToNormalBrake() = 0;
	virtual bool switchToEmergencyLand() = 0;
};


#endif /* CLASSINTERFACES_HPP_ */
