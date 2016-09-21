/*
 * PowerFourLinearTrajectoryFlyTo.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#include "PowerFourLinearTrajectoryFlyTo.hpp"

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::PowerFourLinearTrajectoryFlyTo, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {
  
  
  
    PowerFourLinearTrajectory::PowerFourLinearTrajectory(){
    
    std::cout << "constructor" << std::endl;
      pos_0 = Position3D(0.0,0.0,0.0);
      pos_f = Position3D(0.0,0.0,0.0);
      
      sf = 0.0;
      
      direction = Position3D(0.0,0.0,0.0);
    
      T1 = 0.0;
      dT = 0.0;
      T2 = 0.0;

      a1 = 0.0;
      b1 = 0.0;
      c1 = 0.0;
      d1 = 0.0;
      e1 = 0.0;

      a2 = 0.0;
      b2 = 0.0;
      c2 = 0.0;
      d2 = 0.0;
      e2 = 0.0;

      vm = 0.0;
      k = 0.0;
    std::cout << "parameters initialized to 0.0" << std::endl;
  }
  
  
  
  
  PowerFourLinearTrajectory::PowerFourLinearTrajectory(double t_1, double t_2, double t_3){
    
    std::cout << "constructor" << std::endl;
    
      T1 = t_1;
      dT = t_2;
      T2 = t_3;

      a1 = 0.0;
      b1 = 0.0;
      c1 = 0.0;
      d1 = 0.0;
      e1 = 0.0;

      a2 = 0.0;
      b2 = 0.0;
      c2 = 0.0;
      d2 = 0.0;
      e2 = 0.0;

      vm = 0.0;
      k = 0.0;
  }
  
  
  

  PowerFourLinearTrajectory::~PowerFourLinearTrajectory(){
    std::cout << "destructor" << std::endl;
    
  }
      
      
  void PowerFourLinearTrajectory::compute_traj_parameters(Position3D p0, Position3D pf, double s0, double v0, double max_vel, double max_acc){
    std::cout << "compute_traj_parameters" << std::endl;
    
    pos_0 = p0;
    pos_f = pf;
    
    sf = (pos_f-pos_0).norm();
    
    if (sf>0.01){
      direction = (pos_f-pos_0) / sf;
    }
    else{
      direction = Eigen::VectorXd::Zero(3);
    }
    
    
    double a, b, c, radix_1, radix_2;
    double T1_a, T2_a, T1_b, T2_b, r;
    int two_phases;
  
    double sign = sf-s0;
    if (sign >= 0.0){
      sign=1.0;
    } else {
      sign=-1.0;
    }

    if ((max_vel!=0) && (s0!=sf)){ //check...

      vm = max_vel*sign;
      k = max_acc*sign;

      T1 = -3.0/2.0 * (v0-vm) / k;
      a1 = -4.0/27.0 * pow(k,3) / ((v0-vm)*(v0-vm));
      b1 = -4.0/9.0 * pow(k,2) / (v0-vm);
      e1 = s0;
      d1 = v0;
      c1 = 0.0;

      c2 = 0.0;
      T2 = 3.0/2.0 * vm / k;
      dT = 1.0/4.0 * (-3.0*pow(vm,2) + 4.0*sf*k - 4.0*a1*pow(T1,4)*k - 4.0*b1*pow(T1,3)*k - 4.0*c1*pow(T1,2)*k - 4.0*d1*T1*k - 4.0*e1*k )/vm/k;
      a2 = 4.0/27.0*pow(k,3)/pow(vm,2);
      d2 = vm;
      e2 = 1.0/4.0*(-3.0*pow(vm,2) + 4.0*sf*k) / k;
      b2 = -4.0/9.0 * pow(k,2) /vm;

      if (dT<0) {
        c2 = 0.0;
        c1 = 0.0;

        a=2.0;
        b=12.0*v0;
        c=9.0*pow(v0,2) - 12.0*sf*k + 12.0*s0*k;

        radix_1=(-b + sqrt(pow(b,2) - 4.0*a*c))/(2.0*a);
        radix_2=(-b - sqrt(pow(b,2) - 4.0*a*c))/(2.0*a);
        
//         %decide the correct root
        T1_a = 1.0/2.0* radix_1 / k;
        T2_a = 1.0/2.0* (radix_1 +3.0*v0)/k;
    
        T1_b = 1.0/2.0*radix_2/k;
        T2_b = 1.0/2.0*(radix_2+3.0*v0)/k;
    
       
        
        if (T1_a>=0.0 && T2_a>=0.0) {
	  r=radix_1;

	  T1=T1_a;
	  T2=T2_a;
	  dT=0.0;

	  two_phases=1;
	}
        else if ( T1_b>=0.0 && T2_b>=0.0) {
	  r=radix_2;

	  T1=T1_b;
	  T2=T2_b;
	  dT=0.0;

	  two_phases=1;
	}
        else {
	  r=0;
	  two_phases=0;
	}

        if (two_phases==0) {

	  T1=0.0;
	  T2=2.0*(sf-s0)/v0;
	  dT=0.0;
	  b2=-1.0/4.0*pow(v0,3)/pow(sf-s0, 2);
	  c2=0.0;
	  a2=1.0/16.0*pow(v0,4)/pow(sf-s0, 3);
	  d2=v0;
	  e2=s0;
	}
        else {
	  b1 = -8.0/9.0*pow(k,2)/(4.0*v0*r + 3.0*pow(v0,2) - 4.0*sf*k + 4.0*s0*k)*r;

	  a1 = 8.0/9.0*pow(k,3)/(4.0*v0*r + 3.0*pow(v0,2) - 4.0*sf*k + 4.0*s0*k);

	  b2 = -8.0/9.0*pow(k,2)/(4.0*sf*k + 3.0*pow(v0,2) - 4.0*s0*k)*(r+3.0*v0);
	  e2 = -1.0/8.0*(3.0*pow(v0,2) - 4.0*s0*k - 4.0*sf*k)/k;
	  e1 = s0;
	  d1 = v0;
	  d2 = 2.0/9.0/(4.0*sf*k + 3.0*pow(v0,2) - 4.0*s0*k)*pow(r+3.0*v0, 3);
	  a2 = 8.0/9.0*pow(k,3)/(4.0*sf*k + 3.0*pow(v0,2) - 4.0*s0*k);
        }
      }
    }
  }
  
  
  void PowerFourLinearTrajectory::print(){
    std::cout << T1 << " " << dT << " " << T2 << " " << a1 << " " << b1 << " " << c1 << " " << d1 << " " << e1 << " " << a2 << " " << b2 << " " << c2 << " " << d2 << " " << e2 << " " << vm << " " << k << std::endl;
    
  }
  
  
  void PowerFourLinearTrajectory::interpolator(double t, Position3D &p, Velocity3D &dp, Acceleration3D &ddp, Eigen::Vector3d &dddp){
//     std::cout << " interpolator " << t << std::endl;
    
    Eigen::VectorXd traj = Eigen::VectorXd::Zero(5);
    
    
    if (vm!=0){

      if (t<=T1){
	traj(0)= a1*pow(t,4)+b1*pow(t,3)+c1*pow(t,2)+d1*t+e1;
	traj(1)= 4.0*a1*pow(t,3)+3.0*b1*pow(t,2)+2.0*c1*t+d1;
	traj(2)=12.0*a1*pow(t,2)+6.0*b1*t+2.0*c1;
	traj(3)=24.0*a1*t+6.0*b1;
	traj(4)=1.0;
      }
      else if (t<=T1+dT){
	t=t-T1;

	traj(0)=a1*pow(T1,4)+b1*pow(T1,3)+c1*pow(T1,2)+d1*T1+e1+vm*t;
	traj(1)=vm;
	traj(2)=0.0;
	traj(3)=0.0;
	traj(4)=1.0;
      }
      else if (t<=T1+dT+T2){
        t=t-T1-dT;

        traj(0)=a2*pow(t,4)+b2*pow(t,3)+c2*pow(t,2)+d2*t+e2;
	traj(1)=4.0*a2*pow(t,3)+3.0*b2*pow(t,2)+2.0*c2*t+d2;
	traj(2)=12.0*a2*pow(t,2)+6.0*b2*t+2.0*c2;
	traj(3)=24.0*a2*t+6.0*b2;
	traj(4)=1.0;
      }
      else{
	t=T2;
	traj(0)=a2*pow(t,4)+b2*pow(t,3)+c2*pow(t,2)+d2*t+e2;
	traj(1)=0.0;
	traj(2)=0.0;
	traj(3)=0.0;
	traj(4)=0.0;
      }
    }
    else{
	traj(0)=0.0;
	traj(1)=0.0;
	traj(2)=0.0;
	traj(3)=0.0;
	traj(4)=0.0;
    }
    
    p = pos_0 + traj(0)*direction;
    dp = traj(1)*direction;
    ddp = traj(2)*direction;
    dddp = traj(3)*direction;

  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

PowerFourLinearTrajectoryFlyTo::PowerFourLinearTrajectoryFlyTo()
	: Behavior("tk_be_common/PowerFourLinearTrajectoryFlyTo", BehaviorType::Air)
{

}

void PowerFourLinearTrajectoryFlyTo::initialize()
{
	tPowerFourLinearTrajectoryFlyToDestination = addOption<Position3D>("tPowerFourLinearTrajectoryFlyToDestination",
			"Specifies the Destination of the PowerFourLinearTrajectoryFlyTo Behavior",
			Position3D(0.0,0.0,-1.0), false, false);
	tPowerFourLinearTrajectoryFlyToVelocity = addOption<double>("tPowerFourLinearTrajectoryFlyToVelocity",
			"Defines the Velocity of the PowerFourLinearTrajectoryFlyTo Behavior",
			1.0, false, false);
	tPowerFourLinearTrajectoryFlyToAcceleration = addOption<double>("tPowerFourLinearTrajectoryFlyToAcceleration",
			"Specifies the Destination of the PowerFourLinearTrajectoryFlyTo Behavior",
			0.5, false, false);
	tPowerFourLinearTrajectoryFlyToTime = addOption<double>("tPowerFourLinearTrajectoryFlyToTime",
			"Specifies the Destination of the PowerFourLinearTrajectoryFlyTo Behavior",
			40.0, false, false);
	tPowerFourLinearTrajectoryFlyToDestinationRadius = addOption<double>("tPowerFourLinearTrajectoryFlyToDestinationRadius",
			"Defines the Radius from the Destination, at which the PowerFourLinearTrajectoryFlyTo Behavior turns invalid",
			0.01, false, false);

	//parameterInitialized = true; // not parameter Initialized by default.
}

void PowerFourLinearTrajectoryFlyTo::destroy()
{

}

bool PowerFourLinearTrajectoryFlyTo::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{ 
  	snap = Eigen::Vector3d::Zero();

	start = currentState.position;
	end = tPowerFourLinearTrajectoryFlyToDestination->getValue();
	yawAngle = 0.0;//currentState.getEulerRPY()(2);
	
	linearTrajectory.compute_traj_parameters(start, end, 0.0, 0.0, tPowerFourLinearTrajectoryFlyToVelocity->getValue(), tPowerFourLinearTrajectoryFlyToAcceleration->getValue());
	timer.reset();
	
	//ROS_INFO("PowerFourLinearTrajectoryFlyTo: Storing yawAngle: %f ", yawAngle);
	return true;
}

void PowerFourLinearTrajectoryFlyTo::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void PowerFourLinearTrajectoryFlyTo::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void PowerFourLinearTrajectoryFlyTo::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void PowerFourLinearTrajectoryFlyTo::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	trajectoryStepActive(currentState, generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void PowerFourLinearTrajectoryFlyTo::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	double elapsed = timer.getElapsed().toDSec()/* + 1.0*/ ;
	linearTrajectory.interpolator(elapsed, nextPosition, nextVelocity, nextAcceleration, jerk);
  
	generatedTrajInput.setPosition( nextPosition, nextVelocity, nextAcceleration, jerk, snap);
	generatedTrajInput.setYawAngle( yawAngle );
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void PowerFourLinearTrajectoryFlyTo::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool PowerFourLinearTrajectoryFlyTo::isValid(const TKState& currentState) const
{
	Vector3D direction = tPowerFourLinearTrajectoryFlyToDestination->getValue() - currentState.position;
	return direction.norm() > tPowerFourLinearTrajectoryFlyToDestinationRadius->getValue();
}


} /* namespace telekyb_behavior */
