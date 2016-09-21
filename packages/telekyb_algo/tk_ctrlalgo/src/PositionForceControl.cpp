/*
 * PositionForceControl.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include <tk_ctrlalgo/PositionForceControl.hpp>
#include <algorithm>
#include <geometry_msgs/Vector3.h>

#include <telekyb_defines/physic_defines.hpp>

#define MAX_INT_TIME_STEP 0.04

namespace TELEKYB_NAMESPACE {

// Options
PositionForceControlOptions::PositionForceControlOptions()
    : OptionContainer("PositionForceControl")
{
    tPropGain = addOption<double>( "tPropGain", "Proportional gain on xy plane", 9.0, false, false);
    tXPropGain = addOption<double>( "tXPropGain", "Proportional gain on x axis", 1.0, false, false);
    tYPropGain = addOption<double>( "tYPropGain", "Proportional gain on y axis", 1.0, false, false);
    tDerivGain = addOption<double>( "tDerivGain", "Derivative  gain on xy plane", 7.5, false, false);
    tXDerivGain = addOption<double>( "tXDerivGain", "Derivative  gain on X axis", 1.0, false, false);
    tYDerivGain = addOption<double>( "tYDerivGain", "Derivative  gain on y axis", 1.0, false, false);
    tIntegGain = addOption<double>( "tIntegGain", "Integral  gain on xy plane", 4.5, false, false);
    tXIntegGain = addOption<double>( "tXIntegGain", "Integral  gain on x axis", 1.0, false, false);
    tYIntegGain = addOption<double>( "tYIntegGain", "Integral  gain on y axis", 1.0, false, false);

    tIntegVelGain = addOption<double>( "tIntegVelGain", "Velocity Integral gain on xy plane", 0.0, false, false); // A good choice for this could be 0.8
    tDerGainVelMode = addOption<double>( "tDerGainVelMode", "Derivative  gain on xy plane in velocity mode", 7.5, false, false);

    tZIntegGain = addOption<double>( "tZIntegGain", "Z axis integral gain", 0.0, false, false);
    tZPropGain = addOption<double>( "tZPropGain", "Z axis Proportional gain", 9.0, false, false);
    tZDerivGain = addOption<double>( "tZDerivGain", "Z axis Derivative  gain", 7.5, false, false);

    tSatIntTerm = addOption<double>( "tSatIntTerm",
                                     "Saturation value on the integral term  (sine of the maximum inclination angle) (on xy plane)", 0.475, false, false);
    tSatIntVelTerm = addOption<double>( "tSatIntVelTerm",
                                        "Saturation value on the integral of the velocity term  (sine of the maximum inclination angle) (on xy plane)", 0.0, false, false); // A good choice for this could be 1.0

    tMinThrust = addOption<double>( "tMinThrust", "Mimimum thrust", 5.0, false, true);

    tMaxRollSin = addOption<double>( "tMaxRollSin", "Sinus of the max roll", sin(30.0*M_PI/180.0), false, true);
    tMaxPitchSin = addOption<double>( "tMaxPitchSin", "Sinus of the max pitch", sin(30.0*M_PI/180.0), false, true);

    //tMaxVelInVelMode = addOption<double>( "tMaxVelInVelMode", "Maximum velocity in velocity mode", 100.0, false, true);

    tGravity = addOption<double>( "tGravity", "Gravity Value for the Position Controller", GRAVITY, false, false);
    //	tPubCommandedAcc = addOption<bool>("tPubCommandedAcc",
    //				"Publish commanded accelerations", false, false, true);
}

PositionForceControl::PositionForceControl() {
    time = 0.0;
    xIntErr = 0.0;
    yIntErr = 0.0;
    zIntErr = 0.0;
    yawRateErr = 0.0;
    xIntVelErr = 0.0;
    yIntVelErr = 0.0;
    zIntVelErr = 0.0;

    fileTemp1 = NULL;
    fileTemp2 = NULL;
    //XXX: this is not good practice since it might accidentally report accs of the wrong vehicle!
    //  accPublisher = mainNodehandle.advertise<geometry_msgs::Vector3>("/TeleKyb/TeleKybCore_7/comAcc", 1);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

PositionForceControl::~PositionForceControl()
{

}

//void PositionForceControl::setMass(double mass_)
//{
//	mass = mass_;
//}

void PositionForceControl::run(const TKTrajectory& input, const TKState& currentState, const double mass, PosCtrlOutput& output)
{

    bool printComZ = true;
    bool printComZWithoutIntegralTerm = true;


    time = ros::Time::now().toSec();


    // Current State
    Position3D curPosition = currentState.position;
    Velocity3D curLinVelocity = currentState.linVelocity;
    Vector3D curOrientation = currentState.getEulerRPY();
    //Velocity3D curAngVelocity = currentState.angVelocity;
    

    ROS_DEBUG_STREAM("input position " << input.position(0) << " " << input.position(1) << " " << input.position(2));
    ROS_DEBUG_STREAM("input velocity " << input.velocity(0) << " " << input.velocity(1) << " " << input.velocity(2));

    // Desired State
    Position3D desPosition = input.position;
    Velocity3D desLinVelocity = input.velocity;
    Acceleration3D desLinAcceleration = input.acceleration;
    // NOTE we will use the snap field to pass the additional force
    Vector3D additionalForce = input.jerk;
    

    // if there is a transformation between the Trajectory and the currentState
    //
    tf::StampedTransform transform;
    try{
        // lookup the tf transform between the input and the currentState frame
        // where the currentState frame is the parent frame
        tfListener.lookupTransform(currentState.frame_id,input.frame_id,
                                 ros::Time(0), transform);

        tf::Vector3 positionVector;

        // transfrom the input position into the state frame
        positionVector.setX(desPosition(0) + transform.getOrigin().x());
        positionVector.setY(desPosition(1) + transform.getOrigin().y());
        positionVector.setZ(desPosition(2) + transform.getOrigin().z());

        tf::Vector3 velocityVector;
        // create a tf quaternion from the transformation rotation
        tf::Quaternion bq(transform.getRotation());

        // get the roll pitch and yaw with the provided tf function
        double roll, pitch, yaw;
        tf::Matrix3x3(bq).getRPY(roll, pitch, yaw);


        // transform the velocity vector into the currentState frame
        velocityVector.setX(cos(-yaw) * desLinVelocity(0) - sin(-yaw) * desLinVelocity(1));
        velocityVector.setY(sin(-yaw) * desLinVelocity(0) + cos(-yaw) * desLinVelocity(1));
        velocityVector.setZ(desLinVelocity(2));


        // set the transfromed desired/input position
        desPosition(0) = positionVector.x();
        desPosition(1) = positionVector.y();
        desPosition(2) = positionVector.z();

        // set the transformed velocity
        desLinVelocity(0) = velocityVector.x();
        desLinVelocity(1) = velocityVector.y();
        desLinVelocity(2) = velocityVector.z();

    }
    catch (tf::TransformException ex){
        //ROS_ERROR("%s",ex.what());
    }



    ROS_DEBUG_STREAM("curPosition " << curPosition(0) << " " << curPosition(1) << " " << curPosition(2));
    ROS_DEBUG_STREAM("desPosition " << desPosition(0) << " " << desPosition(1) << " " << desPosition(2));
    ROS_DEBUG_STREAM("desLinVelocity " << desLinVelocity(0) << " " << desLinVelocity(1) << " " << desLinVelocity(2));
    ROS_DEBUG_STREAM("curOrientation " << curOrientation(0) << " " << curOrientation(1) << " " << curOrientation(2));

    double xPropGain = 0.0, yPropGain = 0.0, zPropGain = 0.0;
    double xIntegGain = 0.0, yIntegGain = 0.0, xIntegVelGain = 0.0, yIntegVelGain = 0.0, zIntegVelGain = 0.0, zIntegGain = 0.0;
    double xDerivGain = 0.0, yDerivGain = 0.0, zDerivGain = 0.0;


    if (input.xAxisCtrl == PosControlType::Position) {
        // 		std::cout << "A" << std::endl;
        //std::cout << options.tXPropGain->getValue() << " " << options.tXDerivGain->getValue() << " " << options.tXIntegGain->getValue() << std::endl;
        //std::cout << options.tPropGain->getValue() << " " << options.tDerivGain->getValue() << " " << options.tIntegGain->getValue() << std::endl;
        xPropGain   = options.tPropGain->getValue()*options.tXPropGain->getValue();
        xDerivGain  = options.tDerivGain->getValue()*options.tXDerivGain->getValue();
        xIntegGain  = options.tIntegGain->getValue()*options.tXIntegGain->getValue();
    } else if (input.xAxisCtrl == PosControlType::Velocity) {
        //		xPropGain   = 0.0;
        // 		std::cout << "B" << std::endl;
        xDerivGain  = options.tDerGainVelMode->getValue();
        xIntegVelGain = options.tIntegVelGain->getValue();
        //		xIntegGain  = 0.0;
        xIntErr = 0.0;
    } else if (input.xAxisCtrl == PosControlType::Acceleration) {
        // 		std::cout << "C" << std::endl;
        //		xPropGain   = 0.0;
        //		xDerivGain  = 0.0;
        //		xIntegGain  = 0.0;
        xIntErr = 0.0;
    }

    if (input.yAxisCtrl == PosControlType::Position) {
        // 		std::cout << "A1" << std::endl;
        yPropGain   = options.tPropGain->getValue()*options.tYPropGain->getValue();
        yDerivGain  = options.tDerivGain->getValue()*options.tYDerivGain->getValue();
        yIntegGain  = options.tIntegGain->getValue()*options.tYIntegGain->getValue();
    } else if (input.yAxisCtrl == PosControlType::Velocity) {
        //		yPropGain   = 0.0;
        // 		std::cout << "B1" << std::endl;
        yDerivGain  = options.tDerGainVelMode->getValue();
        yIntegVelGain = options.tIntegVelGain->getValue();
        //		yIntegGain  = 0.0;
        yIntErr = 0.0;
    } else if (input.yAxisCtrl == PosControlType::Acceleration) {
        //		yPropGain   = 0.0;
        //		yDerivGain  = 0.0;
        //		yIntegGain  = 0.0;
        yIntErr = 0.0;
    }

    if (input.zAxisCtrl == PosControlType::Position) {
        // 		std::cout << "A2" << std::endl;
        zIntegGain  = options.tZIntegGain->getValue();
        zPropGain  = options.tZPropGain->getValue();
        zDerivGain = options.tZDerivGain->getValue();
    } else if (input.zAxisCtrl == PosControlType::Velocity) {
        //		zPropGain  = 0.0;
        // 		std::cout << "B2" << std::endl;
        zDerivGain = options.tZDerivGain->getValue();
        zIntegVelGain = options.tIntegVelGain->getValue();
        zIntErr = 0.0;
    } else if (input.zAxisCtrl == PosControlType::Acceleration) {
        //		zPropGain  = 0.0;
        //		zDerivGain = 0.0;
        zIntErr = 0.0;
    }


    //Decimal mass  = _mass;

    //Decimal gravity = getGravity();
    double maxIntTerm = options.tSatIntTerm->getValue() / options.tIntegGain->getValue();
    double maxIntVelTerm = options.tSatIntVelTerm->getValue() / options.tIntegVelGain->getValue();

    //double maxIntTerm = /*0.475*/getSatIntTerm() / getIntGain();

    double comX = desLinAcceleration(0) +
            xDerivGain* (desLinVelocity(0) - curLinVelocity(0)) +
            xPropGain* (desPosition(0) - curPosition(0)) +
            xIntegGain*(xIntErr) +
            xIntegVelGain*(xIntVelErr);
    // 	std::cout << desLinVelocity(0) << "  " << curLinVelocity(0) << std::endl;

    double comY = desLinAcceleration(1) +
            yDerivGain* (desLinVelocity(1) - curLinVelocity(1)) +
            yPropGain* (desPosition(1) - curPosition(1)) +
            yIntegGain*(yIntErr) +
            yIntegVelGain*(yIntVelErr);



    double comZ = desLinAcceleration(2) +
            zDerivGain*(desLinVelocity(2) - curLinVelocity(2)) +
            zPropGain*(desPosition(2) - curPosition(2)) +
            zIntegGain*(zIntErr) +
            zIntegVelGain*(zIntVelErr);
    //XXX: this is not good practice since it might accidentally report accs of the wrong vehicle!
    //	if (options.tPubCommandedAcc->getValue()) {
    //		geometry_msgs::Vector3 comAcc;
    //		comAcc.x = comX;
    //		comAcc.y = comY;
    //		comAcc.z = comZ;
    //		accPublisher.publish(comAcc);
    //	}
    if (printComZ){
        // 		std::cout << "writing on file.. " << time << std::endl;
        // 		fprintf(fileTemp1,"%f %f \n",time, comZ);
    }

    if (printComZWithoutIntegralTerm){
        // 		std::cout << "writing on file.. " << time << std::endl;
        // 		fprintf(fileTemp2,"%f %f \n",time, comZ - zIntegVelGain*(zIntegVelGain));
    }


    // 		std::cout << "desLin acc  " << desLinAcceleration(2) << std::endl;
    // 		std::cout << "derviGa     " << zDerivGain << std::endl;
    // 		std::cout << "desLinV     " << desLinVelocity(2)<< std::endl;
    // 		std::cout << "cur lin vel " << curLinVelocity(2) << std::endl;
    // 		std::cout << "zprop       " << zPropGain << std::endl;
    // 		std::cout << "des pos     " << desPosition(2) << std::endl;
    // 		std::cout << "cur pos     " << curPosition(2) << std::endl;



    //	if(_logFile != 0){
    //		fprintf(_logFile,"%lf %lf %lf \n", desVelX - velX, desVelY - velY, desVelZ - velZ);
    //	}

    double timeStep = integTimer.getElapsed().toDSec();
    integTimer.reset();
    if (timeStep < MAX_INT_TIME_STEP) {
        xIntErr += (desPosition(0) - curPosition(0))*timeStep;
        yIntErr += (desPosition(1) - curPosition(1))*timeStep;
        zIntErr += (desPosition(2) - curPosition(2))*timeStep;
        xIntVelErr += (desLinVelocity(0) - curLinVelocity(0))*timeStep;
        yIntVelErr += (desLinVelocity(1) - curLinVelocity(1))*timeStep;
        zIntVelErr += (desLinVelocity(2) - curLinVelocity(2))*timeStep;
    }

    if (fabs(xIntErr) > maxIntTerm) {
        xIntErr = copysign(maxIntTerm, xIntErr);
        //xIntErr = DecimalUtilities::sign(xIntErr)*maxIntTerm;
    }

    if (fabs(yIntErr) > maxIntTerm) {
        yIntErr = copysign(maxIntTerm, yIntErr);
        //yIntErr=DecimalUtilities::sign(yIntErr)*maxIntTerm;
    }

    if (fabs(zIntErr) > maxIntTerm) {
        zIntErr = copysign(maxIntTerm, zIntErr);
        //zIntErr=DecimalUtilities::sign(zIntErr)*maxIntTerm;
    }

    if (fabs(xIntVelErr) > maxIntVelTerm) {
        xIntVelErr = copysign(maxIntVelTerm, xIntVelErr);
        //yIntErr=DecimalUtilities::sign(yIntErr)*maxIntTerm;
    }

    if (fabs(yIntVelErr) > maxIntVelTerm) {
        yIntVelErr = copysign(maxIntVelTerm, yIntVelErr);
        //yIntErr=DecimalUtilities::sign(yIntErr)*maxIntTerm;
    }

    if (fabs(zIntVelErr) > maxIntVelTerm) {
        zIntVelErr = copysign(maxIntVelTerm, zIntVelErr);
        //yIntErr=DecimalUtilities::sign(yIntErr)*maxIntTerm;
    }


    // 	std::cout << "the command on x is : " << comX << ", the command on y is : " << comY << std::endl;
    // 	std::cout << "the integral error on x is : " << xIntVelErr << ", the integral error on y is : " << yIntVelErr << std::endl;
    // 	std::cout << "the gain is: " << xIntegVelGain << std::endl;
    //	double rollCos = cos(curOrientation(0));
    //	double pitchCos = cos(curOrientation(1));

    //	if(fabs(rollCos) <= MIN_COS){
    //		rollCos = copysign(MIN_COS, rollCos);
    //		//rollCos = DecimalUtilities::sign(rollCos) * MIN_COS;
    //	}
    //
    //	if(fabs(pitchCos) <= MIN_COS){
    //		pitchCos = copysign(MIN_COS, pitchCos);
    //		//pitchCos = DecimalUtilities::sign(pitchCos) * MIN_COS;
    //	}

    double comThrust =  ( mass/(cos( (double)curOrientation(0) )*cos( (double)curOrientation(1) )) ) * ( -1.0 * options.tGravity->getValue() + comZ - input.jerk[2]);
    // 	std::cout << "mass " <<  mass << "  comZ " <<  comZ <<  std::endl;



    /*saturation min/max thrust*/
    //	if (comThrust > -getMinThrust()){
    //		comThrust = -getMinThrust();
    //	}
    //
    //	if (comThrust < -2.0*mass * gravity + getMinThrust()){
    //		comThrust = -2.0*mass * gravity + getMinThrust();
    //	}

    comThrust = std::min( comThrust, -1.0 * options.tMinThrust->getValue() );
    comThrust = std::max( comThrust, -2.0 * mass * options.tGravity->getValue() + options.tMinThrust->getValue() );



    //	double comRoll = ( mass / comThrust) * ( -sin(yaw)*comX + cos(yaw)*comY);
    //	double comPitch = ( mass / (comThrust*cos(roll))) * ( cos(yaw)*comX + sin(yaw)*comY);
    double comRoll = ( mass / comThrust) * ( -sin((double)curOrientation(2))*(comX-input.jerk[0]) + cos((double)curOrientation(2))*(comY-input.jerk[1]));

    double comPitch = ( mass / (comThrust*cos((double)curOrientation(0)))) * ( cos((double)curOrientation(2))*(comX-input.jerk[0]) + sin((double)curOrientation(2))*(comY-input.jerk[1]));


    if (fabs(comRoll) > options.tMaxRollSin->getValue()){
        //		comRoll = DecimalUtilities::sign(comRoll) * getMaxRollSin();
        comRoll = copysign(options.tMaxRollSin->getValue(), comRoll);
    }

    if (fabs(comPitch) > options.tMaxPitchSin->getValue()){
        //		comPitch = DecimalUtilities::sign(comPitch) * getMaxPitchSin();
        comPitch = copysign(options.tMaxPitchSin->getValue(), comPitch);
    }

    //	std::cout << "comRoll " << comRoll  << " " << options.tMaxPitchSin->getValue() << std::endl;
    output.comRoll = -asin( comRoll );
    output.comPitch = asin( comPitch );
    output.comThrust = comThrust;
    ROS_DEBUG_STREAM_THROTTLE(1,"tk ctrlAlgo PositionForceControl comRoll " << output.comRoll);
    ROS_DEBUG_STREAM_THROTTLE(1,"tk ctrlAlgo PositionForceControl comPitch " << output.comPitch);


    //	out->commands.clear();
    //	out->commands.reserve(4*sizeof(double));
    //	out->commands.push_back(comRoll);
    //	out->commands.push_back(comPitch);
    //	out->commands.push_back(comThrust);
}


}
