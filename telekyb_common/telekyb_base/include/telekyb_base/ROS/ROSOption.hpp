/*
 * ROSOption.hpp
 *
 *  Created on: Oct 17, 2011
 *      Author: mriedel
 */

#ifndef ROSOPTION_HPP_
#define ROSOPTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/ROS/ROSOptionController.hpp>
#include <telekyb_base/Tools/XmlRpcConversion.hpp>

// For Parsing YAML.
#include <telekyb_base/Tools/YamlHelper.hpp>

// For missing comparision
#include <telekyb_base/Tools/OperatorOverload.hpp>

// For Setting Options via YAML.
#include <telekyb_srvs/StringInput.h>
#include <telekyb_srvs/StringOutput.h>

namespace TELEKYB_NAMESPACE
{

template < class _T >
class ROSOption : public ROSBaseOption, public OptionListener<_T>
{
protected:
	//Reference to acutal Option
	Option<_T>* option;
	_T valueOnRos;

	// -------------- SERVICES ------------- //
	// Service Offered to change Option
	ros::ServiceServer getService;
	ros::ServiceServer setService;

	bool getServiceCallBack(
			telekyb_srvs::StringOutput::Request& request,
			telekyb_srvs::StringOutput::Response& response) {
		ROS_INFO_STREAM("SYNC: Option " << option->getNSName() << " received getSerivce . Answered: " << YamlHelper::parseValueToString(option->getValue()));

		return YamlHelper::parseValueToString(option->getValue(), response.output);
	}

	bool setServiceCallBack(
			telekyb_srvs::StringInput::Request& request,
			telekyb_srvs::StringInput::Response& response) {
		//ROS_INFO_STREAM("SYNC: Option " << option->getNSName() << " Callback with String: " << request.input);

		//_T value;
		YAML::Node node;
		if ( ! YamlHelper::parseStringToNode(request.input, node)) {
			ROS_INFO_STREAM("SYNC: Option " << option->getNSName() << " unable to convert from Yaml." );
			return false;
		}

		return option->set(node);

//		if (value == option->getValue()) {
//			// value unchanged
//			ROS_INFO_STREAM("SYNC: Option " << option->getNSName() << " is already at the defined value!" );
//			return true;
//		}
		// value changed

//		if (option->isReadOnly()) {
//			// readOnly! valid on Ros differs, but should NOT. Revert.
//			ROS_WARN_STREAM("SYNC: Read-only Option " << option->getNSName() << " cannot be set!");
//			return false;
//		}
//		// option writeable
//
//		// bounds
//		if (option->hasBounds() && !option->isWithinBounds(value)) {
//			ROS_WARN_STREAM("SYNC: Cannot set Option " << option->getNSName() << ". Value out-of-bounds !");
//			return false;
//		}

		// set
//		ROS_INFO_STREAM("SYNC: Updating Option " << option->getNSName() << " to " << YamlHelper::parseValueToString(value));
//		option->setValue(value);
//		return true;
	}

	// Creates the ROS Service to Get the Parameter with YAML Syntax.
	virtual void createGetService() {
		// This should never happen.
		if (!ROSOptionController::hasInstance()) {
			ROS_ERROR_STREAM("createSetService() called on ROSOption " << option->getNSName() << ", ROSOptionController is not initialized yet.");
			return;
		}
		ros::NodeHandle serviceHandle(ROSOptionController::Instance().getOptionNodeHandle(), getNSName());
		getService = serviceHandle.advertiseService(OPTION_GETSERVICE_NAME, &ROSOption::getServiceCallBack, this);
	}

	// Creates the ROS Service to Set the Parameter with YAML Syntax.
	virtual void createSetService() {
		// This should never happen.
		if (!ROSOptionController::hasInstance()) {
			ROS_ERROR_STREAM("createSetService() called on ROSOption " << option->getNSName() << ", ROSOptionController is not initialized yet.");
			return;
		}
		ros::NodeHandle serviceHandle(ROSOptionController::Instance().getOptionNodeHandle(), getNSName());
		setService = serviceHandle.advertiseService(OPTION_SETSERVICE_NAME, &ROSOption::setServiceCallBack, this);
	}

	virtual void shutdownGetService() {
		getService.shutdown();
	}

	virtual void shutdownSetService() {
		setService.shutdown();
	}

	// -------------- SERVICES ------------- //

	// Parameter Server Interaction
	virtual void setToParameterServer() {
		if (!ROSOptionController::hasInstance()) {
			ROS_ERROR_STREAM("setToParameterServer() called on ROSOption " << option->getNSName() << ", ROSOptionController is not initialized yet.");
			return;
		}
		XmlRpc::XmlRpcValue value;
		// convert
		//ToXmlRpcConversion<_T> converter;
		//converter(option->getValue(), value);
		try {
			option->getValue() >> value;
			ros::NodeHandle optionHandle = ROSOptionController::Instance().getOptionNodeHandle();
			optionHandle.setParam(option->getNSName(), value);
			valueOnRos = option->getValue();
		} catch (XmlRpc::XmlRpcException &e) {
			ROS_ERROR_STREAM("Could not convert Option " << option->getNSName() << " to XmlRpcValue! " << e.getMessage());
		}
	}

	virtual bool updateFromParameterServer() {
		if (!ROSOptionController::hasInstance()) {
			ROS_ERROR_STREAM("updateFromParameterServer() called on ROSOption " << option->getNSName() << ", ROSOptionController is not initialized yet.");
			return false;
		}

		ros::NodeHandle optionHandle = ROSOptionController::Instance().getOptionNodeHandle();

		XmlRpc::XmlRpcValue value;
		optionHandle.getParam(option->getNSName(), value);

		// convert
		//FromXmlRpcConversion<_T> converter;


		//ROS_DEBUG_STREAM("updateFromParameterServer on Option " << getName() << " currentValue: " << option->getValue() << " valueOnRos: " << valueOnRos);

		// Conversion
		try {
			value >> valueOnRos;
		} catch (XmlRpc::XmlRpcException &e) {
			// value On Ros is invalid! Revert to Optionvalue.
			ROS_ERROR_STREAM("Could not convert Option " << option->getNSName() << " FROM XmlRpcValue! " << e.getMessage() << " Reverting..");
			setToParameterServer();
			return false;
		}
		// conversion ok.

		if (valueOnRos == option->getValue()) {
			// value unchanged
			return true;
		}
		// value changed

		if (option->isReadOnly()) {
			// readOnly! valid on Ros differs, but should NOT. Revert.
			ROS_WARN_STREAM("ASYNC: Read-only Option " << option->getNSName() << " was changed on ROS Parameter server. Reverting...");
			setToParameterServer();
			return false;
		}
		// option writeable

		// bounds
		if (option->hasBounds() && !option->isWithinBounds(valueOnRos)) {
			ROS_WARN_STREAM("ASYNC: Option " << option->getNSName() << " was changed on ROS Parameter server, but is out of bounds! Reverting...");
			setToParameterServer();
			return false;
		}
		// either no bounds or within bounds

		// finally! let's update the option
		ROS_INFO_STREAM("ASYNC: Updating Option " << option->getNSName() << " to " << YamlHelper::parseValueToString(valueOnRos));
		option->setValue(valueOnRos);

		return true;
	}

	virtual bool deleteFromParameterServer() {
		if (!ROSOptionController::hasInstance()) {
			ROS_ERROR_STREAM("deleteFromParameterServer() called on ROSOption " << option->getNSName() << ", ROSOptionController is not initialized yet.");
//			std::cout << "deleteFromParameterServer() called on ROSOption " << option->getNSName() <<
//					", ROSOptionController is not initialized yet." << std::endl;
			return false;
		}

		//std::cout << "Delete Called on " << option->getNSName() << std::endl;

		ros::NodeHandle optionHandle = ROSOptionController::Instance().getOptionNodeHandle();

		return optionHandle.deleteParam(option->getNSName());
	}

	friend class ROSOptionController;

public:
	ROSOption(Option<_T>* option_) {
		option = option_;
	}
	virtual ~ROSOption() {

	}

	virtual void optionDidChange(const Option<_T>* option_) {
		//Set if Value is differnt
		if(option_->getValue() != valueOnRos) {
			setToParameterServer();
		}
	}
	virtual void optionShouldDelete(const Option<_T>* option_) {
		// Option is never deleted by this notification, but explicitly via the OptionController in the Option<T> Dest.
		//deleteFromParameterServer();
	}



	virtual std::string getName() const {
		return option->getName();
	}
	virtual std::string getDescription() const {
		return option->getDescription();
	}
	virtual std::string getNamespace() const {
		return option->getNamespace();
	}
	virtual std::string getNSName() const {
		return option->getNSName();
	}
//
//	void spinOnce() {
//		std::cout << "Option: " << option->getName() << " Value: " << option->getValue() << std::endl;
//	}

};

} // namespace

#endif /* ROSOPTION_HPP_ */
