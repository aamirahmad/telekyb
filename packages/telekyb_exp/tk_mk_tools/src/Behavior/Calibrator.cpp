/*
 * Calibrator.cpp
 *
 *  Created on: Feb 8, 2012
 *      Author: mriedel
 */

#include "Calibrator.hpp"

#include <telekyb_base/ROS/ROSModule.hpp>


#include <telekyb_base/Options/CommonOptions.hpp>

#define CALIBRATION_YAW_ANGLE 0.0
#define DEFAULT_ACC_OFFSET 512
//#define MAX_ALLOWED_ERROR 0.01
#define VECTOR_ERROR_THRESHOLD 0.005

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::Calibrator, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Calibrator::Calibrator()
	: Behavior("tk_mk_tools/Calibrator", BehaviorType::Air),
	  mkInterface(NULL),
	  offsetRawAcc_X(MKDataDefines::OFFSET_RAW_ACC_X, DEFAULT_ACC_OFFSET),
	  offsetRawAcc_Y(MKDataDefines::OFFSET_RAW_ACC_Y, DEFAULT_ACC_OFFSET)


{

}

void Calibrator::initialize()
{
	tMaxInitialVelocity = addOption<double>("tMaxInitialVelocity",
			"Defines the Maximal Initial Velocity to be able to switch into the Calibrator",
			0.1, false, true);
	tMaxInitialYawError = addOption<double>("tMaxInitialYawError",
			"Defines the Maximal Yaw Error in Radians to switch into the Calibrator",
			0.3, false, true);
	tSettleTime  = addOption<double>("tSettleTime",
			"Settling Time in s for each Calibration step. Includes 2s transition time!",
			10.0, false, true);

	tValueRange = addOption<int>("tValueRange",
			"Values to check around center X/Y. Results in a (2*Range + 1)^2 Matrix",
			50, false, true);
	tCenterValueX = addOption<int>("tCenterValueX",
			"Center Value for X",
			DEFAULT_ACC_OFFSET, false, true);
	tCenterValueY = addOption<int>("tCenterValueY",
			"Center Value for Y",
			DEFAULT_ACC_OFFSET, false, true);


	tPCIntegralGain = OptionContainer::getGlobalOptionByName<double>("PositionControl","tIntegGain");
	tDoMassEstimation = OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoMassEstimation");

	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	// no Parameters
	parameterInitialized = true;
}

void Calibrator::destroy()
{
	// remove Options
	std::cout << "Deleting MKInterface" << std::endl;
	if (mkInterface) {
		delete mkInterface;
	}
	std::cout << "Done Deleting MKInterface" << std::endl;
}

bool Calibrator::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// Velocity can should almost be 0
	if (currentState.linVelocity.norm() > tMaxInitialVelocity->getValue()) {
		ROS_ERROR_STREAM("Too fast to switch into the Calibrator! Current Velocity Norm: "
				<< currentState.linVelocity.norm());
		return false;
	}

	// Yaw Error should be almost 0
	double angleError = fabs(CALIBRATION_YAW_ANGLE - currentState.getEulerRPY()(2) );
	if (angleError > tMaxInitialYawError->getValue()) {
		ROS_ERROR_STREAM("Yaw Angle too big. Angle Error: " << angleError);
		return false;
	}


	// Move RobotID back to CommonOptions???
	Option<int>* robotID = OptionContainer::getGlobalOptionByName<int>("TeleKybCore", "tRobotID");
	if (!robotID) {
		ROS_ERROR_STREAM("Could not get robotID Option");
		return false;
	}

	// Get Interface
	if (!mkInterface) {
		mkInterface = telekyb_interface::MKInterface::getMKInterface(robotID->getValue());
	}
	//mkInterface = telekyb_interface::MKInterface::getMKInterface(1); // BEWARE TEMPORARY!!!
	if (!mkInterface) {
		// fail
		ROS_ERROR("Unable to get MKInterface for UAV with ID %d!", robotID->getValue());
		return false;
	}

	// save position
	calibrationPosition = currentState.position;

	// create Matrix -> setValues NAN
	errorMatrix = Eigen::MatrixXd::Constant(
			2 * tValueRange->getValue() + 1,
			2 * tValueRange->getValue() + 1,
			std::numeric_limits<double>::quiet_NaN());

	// Initial Field -> Middle
	currentField(0) = tValueRange->getValue();
	currentField(1) = tValueRange->getValue();

	// Initial Testfield
	//currentTestOffset = -1; // IMPORTANT AUTOMATIC FIRST INCREASE!
	initialSetup = true;
	setValueThreadDone = true;

	calibrationDone = false;

	// save and put to 0.0;
	tPCIntegralGainSave = tPCIntegralGain->getValue();
	tPCIntegralGain->setValue(0.0);

	// save put off
	tDoMassEstimationSave = tDoMassEstimation->getValue();
	tDoMassEstimation->setValue(false);

	// temp!
	behaviorActiveTimer.reset();
	return true;
}

void Calibrator::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void Calibrator::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// set back
	tPCIntegralGain->setValue(tPCIntegralGainSave);
	tDoMassEstimation->setValue(tDoMassEstimationSave);
}

void Calibrator::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}


// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void Calibrator::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setPosition(calibrationPosition);
	generatedTrajInput.setYawAngle(CALIBRATION_YAW_ANGLE);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Calibrator::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	//std::cout << "Call Calibrator::trajectoryStep" << std::endl;
	// always set to the calibration position with constant YawAngle (here it's 0)
	generatedTrajInput.setPosition(calibrationPosition);
	generatedTrajInput.setYawAngle(CALIBRATION_YAW_ANGLE);


	if (behaviorActiveTimer.getElapsed().toDSec() < 3.0) {
		// do not do anything
		return;
	}

	if (initialSetup) {

		int currentTestValueX = tCenterValueX->getValue() - tValueRange->getValue() +
				currentField(0);

		int currentTestValueY = tCenterValueY->getValue() - tValueRange->getValue() +
				currentField(1);

		ROS_INFO("Now Testing (%d,%d)",
				currentTestValueX,
				currentTestValueY);

		offsetRawAcc_X.value = currentTestValueX;
		offsetRawAcc_Y.value = currentTestValueY;
		valuesSet = false;

//			mkInterface->setMKValueAsync(offsetRawAcc_X);
//			mkInterface->setMKValueAsync(offsetRawAcc_Y);
		// spawn thread
		//ROS_INFO("Creating Thread");
		setValueThreadDone = false;
		setValueThread = boost::thread(&Calibrator::setValueThreadFunc, this);

		initialSetup = false;
	}
	// don't do anthing within the first two seconds // parameterize
//	if (fabs(currentState.getEulerRPY()(2) - CALIBRATION_YAW_ANGLE) > 0.01)
//		ROS_INFO("Yaw not correct yet.");
//		return true;
//	}

	// Calibration logic

	//ROS_INFO("Error: %f", currentError);

	// Catch Setting Thread done
	if (!setValueThreadDone && setValueThread.timed_join(boost::posix_time::microseconds(10))) {
		ROS_INFO("Thread has finished");
		if (!valuesSet) {
			ROS_ERROR("Values could not be set to MK! Canceling Calibration... Implement Retry if this occurs frequently!");
			calibrationDone = true;
		}
		setValueThreadDone = true;

		// set initial conditions
		//maxErrorTest = 0.0;
		//meanErrorTest = 0.0;
		//meanErrorVector = Eigen::Vector2d::Zero();
		//meanNrSamples = 0;
		transitionTimer.reset();
		testTimer.reset();

		// Start new accumulator
		acc = accumulator_set<double, stats<tag::median > >();
		accMedianX = accumulator_set<double, stats<tag::median > >();
		accMedianY = accumulator_set<double, stats<tag::median > >();
	}

	// currently testing?
//	if (activeTest) {


		//std::cout << "Done Checking Thread with Timed_join" << std::endl;

//		if (valuesSet &&
//				mkInterface->getMKCompleteSingeValue(offsetRawAcc_X.id).value == offsetRawAcc_X.value &&
//				mkInterface->getMKCompleteSingeValue(offsetRawAcc_Y.id).value == offsetRawAcc_Y.value) {
//			ROS_ERROR("Values successfully set!");
//			valuesSet = true;
//
//		}
////
		if (setValueThreadDone && valuesSet && transitionTimer.getElapsed().toDSec() > 2.0) {
			Eigen::Vector2d errorVector = (currentState.position.head(2) - calibrationPosition.head(2));
			double currentError = errorVector.norm();

			// Test
			//maxErrorTest = std::max(currentError, maxErrorTest);

			//meanNrSamples++;
			//meanErrorTest = ((double)(meanNrSamples-1) / meanNrSamples)
			//		* meanErrorTest + (currentError / meanNrSamples);

			//meanErrorVector = (double)((meanNrSamples-1) / meanNrSamples) * meanErrorVector +
			//		(errorVector / (double)meanNrSamples);

			acc(currentError);
			accMedianX(errorVector(0));
			accMedianY(errorVector(1));



			// Stop test if timer is done
			if (testTimer.getElapsed().toDSec() > tSettleTime->getValue()) {
				// write fields
				//double result = 1;
				//std::cout << "test" << median(acc) << std::endl;


				errorMatrix(
						currentField(0),
						currentField(1)) = median(acc);

				Eigen::Vector2d medianErrorVector( median(accMedianX) , median(accMedianY) );

//				activeTest = false;
//
				ROS_INFO_STREAM("ErrorMatrix: " << std::endl << errorMatrix.block(currentField(0)-3,currentField(1)-3,7,7));
				//ROS_INFO_STREAM("Mean Error Vector: " << std::endl << meanErrorVector);
				ROS_INFO_STREAM("Median Error Vector: " << std::endl << medianErrorVector);
				// Where to change into


				// X and Y jump
				double constantOffset = 1.0-15.0*sqrt(VECTOR_ERROR_THRESHOLD);
				for (int i = 0; i < 2; ++i) {
					double jumpStep = copysign((sqrt(fabs(medianErrorVector(i))) * 15) + constantOffset, medianErrorVector(i));
					//ROS_INFO("Value for %d: %f", i, jumpStep);
					currentField(i) -= (int)jumpStep;
				}

				int currentTestValueX = tCenterValueX->getValue() - tValueRange->getValue() +
						currentField(0);

				int currentTestValueY = tCenterValueY->getValue() - tValueRange->getValue() +
						currentField(1);

				// We can do this here, because we know(!) there was no field change!
				if (fabs(medianErrorVector(0)) <= VECTOR_ERROR_THRESHOLD &&
						fabs(medianErrorVector(1)) <= VECTOR_ERROR_THRESHOLD) {
					// BREAK Condition
					ROS_WARN("X and Y below Threshold. Calibration done.");
					ROS_WARN("Calibrated Values (X,Y): (%d,%d)", currentTestValueX, currentTestValueY);
					calibrationDone = true;
				}

				ROS_INFO("Now Testing (%d,%d)",
						currentTestValueX,
						currentTestValueY);

				// TODO: Test for Bordercondition


				//ROS_INFO("CurrentOffset: %d", currentTestOffset);

//				if (!isnan(errorMatrix(
//							currentField(0),
//							currentField(1)))) {
//
//					ROS_ERROR("Field already tested. Jumped back?");
//					return false;
//				}


				// set values on QC
				offsetRawAcc_X.value = currentTestValueX;
				offsetRawAcc_Y.value = currentTestValueY;
				valuesSet = false;

	//			mkInterface->setMKValueAsync(offsetRawAcc_X);
	//			mkInterface->setMKValueAsync(offsetRawAcc_Y);
				// spawn thread
				//ROS_INFO("Creating Thread");
				setValueThreadDone = false;
				setValueThread = boost::thread(&Calibrator::setValueThreadFunc, this);
				//ROS_INFO("Done creating Thread");

				// set initial conditions next test
				//activeTest = true;
			}
		}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void Calibrator::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setPosition(calibrationPosition);
	generatedTrajInput.setYawAngle(CALIBRATION_YAW_ANGLE);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Calibrator::isValid(const TKState& currentState) const
{
	// never turns invalid
	return !calibrationDone;
}




// Threaded SetValue
void Calibrator::setValueThreadFunc()
{
	ROS_INFO("In Thread setValueThreadFunc!");
	// set X and Y
	if (mkInterface->setMKValue(offsetRawAcc_X) && mkInterface->setMKValue(offsetRawAcc_Y)) {
		// everything ok
		ROS_INFO("Successfully set values for X and Y to MK");
		valuesSet = true;
	} else {
		// error
		ROS_ERROR("Could not set values to MK!");
	}
	ROS_INFO("Done with Thread setValueThreadFunc!");
}

}
