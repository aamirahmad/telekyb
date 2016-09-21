/*
 * PowerFourLinearTrajectoryFlyTo.hpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#ifndef POWERFOURLINEARTRAJECTORYFLYTO_HPP_
#define POWERFOURLINEARTRAJECTORYFLYTO_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Time.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {
  
  
  
  class PowerFourLinearTrajectory{
    private:
      
      
      Position3D pos_0;
      Position3D pos_f;
      
      double sf;
      
      Position3D direction;
      
      
      // 15 parameters that define the trajectory
      double T1;
      double dT;
      double T2;

      double a1;
      double b1;
      double c1;
      double d1;
      double e1;

      double a2;
      double b2;
      double c2;
      double d2;
      double e2;

      double vm;
      double k;
      
      

    protected:


    public:
      
      
      PowerFourLinearTrajectory();
      
      PowerFourLinearTrajectory(double t_1, double t_2, double t_3);

      ~PowerFourLinearTrajectory();
      
      void compute_traj_parameters(Position3D p0, Position3D pf, double s0, double v0, double max_vel, double max_acc);
      
      void interpolator(double t, Position3D &p, Velocity3D &dp, Acceleration3D &ddp, Eigen::Vector3d &dddp);//   ris=interpolator(t, parameters)
      
      void print();
  };
  
  
  
  
  
  
  
  
  

class PowerFourLinearTrajectoryFlyTo/**/ : public Behavior {
protected:
	Option<Position3D>* tPowerFourLinearTrajectoryFlyToDestination;
	Option<double>* tPowerFourLinearTrajectoryFlyToVelocity;
	Option<double>* tPowerFourLinearTrajectoryFlyToAcceleration;
	Option<double>* tPowerFourLinearTrajectoryFlyToTime;
	Option<double>* tPowerFourLinearTrajectoryFlyToDestinationRadius;
	
	double accelerationTime;
	double constantSpeedTime;
	double yawAngle;
	
	Position3D start;
	Position3D end;
	
	Velocity3D vel;
	Velocity3D acc;
	
	Timer timer;
	
	Position3D nextPosition;
	Velocity3D nextVelocity;
	Acceleration3D nextAcceleration;
	Eigen::Vector3d jerk;
	Eigen::Vector3d snap;
	
	
	PowerFourLinearTrajectory linearTrajectory;

public:
	PowerFourLinearTrajectoryFlyTo();

	virtual void initialize();
	virtual void destroy();

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
};

} /* namespace telekyb_behavior */
#endif /* POWERFOURLINEARTRAJECTORYFLYTO_HPP_ */
