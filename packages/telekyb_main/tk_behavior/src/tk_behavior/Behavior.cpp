/*
 * Behavior.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: mriedel
 */

#include <tk_behavior/Behavior.hpp>

#include <tk_behavior/BehaviorController.hpp>

#include <boost/lexical_cast.hpp>

#include <tk_behavior/BehaviorHelper.hpp>

// For printing
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace TELEKYB_NAMESPACE
{

Behavior::Behavior(const std::string& name_, const BehaviorType& type_)
    : OptionContainer( name_ + "/" + getIDString() ),
      type( type_ ),
      bController( BehaviorController::Instance() ),
      nodeHandle( bController.getBehaviorNodeHandle(), getIDString()),
      name( name_ ),
      parameterInitialized( false ),
      nextBehavior( NULL ),
      behaviorInterface ( NULL )
{
    behaviorInterface = new BehaviorInterface(*this);
    ROS_DEBUG("Creating Behavior ID: %" PRIu64 " Name: %s", getID(), getName().c_str());
}

Behavior::~Behavior()
{
    ROS_DEBUG("Deleting Behavior ID: %" PRIu64 " Name: %s", getID(), getName().c_str());
    delete behaviorInterface;
}

uint64_t Behavior::getID() const
{
    return (uint64_t)this;
}

std::string Behavior::getIDString() const
{
    return boost::lexical_cast<std::string>(getID());
}

bool Behavior::isParameterInitialized() const
{
    return parameterInitialized;
}

void Behavior::setParameterInitialized(bool parameterInitialized_)
{
    parameterInitialized = parameterInitialized_;
}

const ros::NodeHandle& Behavior::getNodeHandle() const
{
    return nodeHandle;
}

//void Behavior::setName(const std::string& name_)
//{
//	name = name_;
//}

std::string Behavior::getName() const
{
    return name;
};

bool Behavior::isActive() const
{
    return (bController.getActiveBehavior() == this);
}

bool Behavior::setNextBehavior(Behavior* nextBehavior_)
{
    // should we allow to set oneself as next Behavior. I think yes! :)

    // validity check
    if ( !exists(nextBehavior_) ) {
        ROS_ERROR_STREAM("Trying to set nextBehavior of " << getName() << " to an invalid BehaviorInstance!");
        return false;
    }

    nextBehavior = nextBehavior_;
    return true;
}

void Behavior::unsetNextBehavior()
{
    nextBehavior = NULL;
}

bool Behavior::hasNextBehavior() const
{
    return (nextBehavior != NULL);
}

Behavior* Behavior::getNextBehavior() const
{
    return nextBehavior;
}

BehaviorType Behavior::getType() const
{
    return type;
}

bool Behavior::canTransitionTo(const Behavior& toBehavior) const
{
    return BehaviorHelper::isAllowedBehaviorTransition(*this, toBehavior);
}

// static Helpers
Behavior* Behavior::behaviorFromID(uint64_t behaviorID)
{
    return (Behavior*)behaviorID;
}

uint64_t Behavior::behaviorToID(Behavior* behavior)
{
    return (uint64_t)behavior;
}
bool Behavior::exists(Behavior *behavior)
{
    return BehaviorContainer::behaviorInstanceExists(behavior);
}
bool Behavior::exists(uint64_t behaviorID)
{
    return  BehaviorContainer::behaviorInstanceExists(behaviorFromID(behaviorID));
}


}

void Behavior::setTrajectoryHeader(TKTrajectory& generatedTrajInput){
    generatedTrajInput.setHeader(ros::Time::now(),"");
}

