/*
 * Joystick.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#include <telekyb_hid/Joystick.hpp>


#include <fstream>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/Joy.h>
#include <telekyb_base/ROS/ROSModule.hpp>

#include <ros/ros.h>

#include <boost/algorithm/string.hpp>

namespace YAML {
	template<>
   struct convert<TELEKYB_NAMESPACE::JoyAxis> {
	      static Node encode(const TELEKYB_NAMESPACE::JoyAxis& rhs) {
	         Node node;
	         node["pos"] = rhs.bytePos;
	         node["name"] = rhs.name;
	         node["min"] = rhs.min;
	         node["max"] = rhs.max;
	         return node;
	      }

	      static bool decode(const Node& node, TELEKYB_NAMESPACE::JoyAxis& rhs) {
	    	  if (node["pos"] && node["name"] && node["min"] && node["max"]) {
				 rhs.bytePos = node["pos"].as<int>();
				 rhs.name = node["name"].as<std::string>();
				 rhs.min = node["min"].as<int>();
				 rhs.max = node["max"].as<int>();
				 return true;
	         }

	         return false;
	      }
   };
}


namespace YAML {
template<>
   struct convert<TELEKYB_NAMESPACE::JoyButton> {
	      static Node encode(const TELEKYB_NAMESPACE::JoyButton& rhs) {
	         Node node;
	         node["pos"] = rhs.bytePos;
	         node["name"] = rhs.name;
	         node["value"] = rhs.value;
	         node["mask"] = rhs.mask;
	         return node;
	      }

	      static bool decode(const Node& node, TELEKYB_NAMESPACE::JoyButton& rhs) {
	    	  if (node["pos"] && node["name"] && node["value"] && node["mask"]) {
	 	         rhs.bytePos = node["pos"].as<int>();
	 	         rhs.name = node["name"].as<std::string>();
	 	         rhs.value = node["value"].as<int>();
	 	         rhs.mask = node["mask"].as<int>();
	 	         return true;
	    	  }
	    	  return false;
	      }
   };
}

namespace TELEKYB_NAMESPACE {

Joystick::Joystick(unsigned short int vendorID_ , unsigned short int productID_, const std::string& name_)
	: HIDDevice(vendorID_, productID_), name(name_)
{
	options = JoystickOptions::InstancePtr();

	nodeHandle = ROSModule::Instance().getNodeNameNodeHandle();

	frameID = getProductString();
	std::string topicName = options->tPubName->getValue();
	if (options->tJoystickUseProductIDForRosPath->getValue()) {
		topicName = getProductString();
		boost::erase_all(topicName, " ");
		boost::erase_all(topicName, "(");
		boost::erase_all(topicName, ")");
		boost::erase_all(topicName, "-");
		ROS_INFO_STREAM("String: " << topicName);
	}
	pub = nodeHandle.advertise<sensor_msgs::Joy>(topicName, 100);

	config.setAxisCutOff(options->tDeadZone->getValue());
}

Joystick::~Joystick()
{

}

void Joystick::startThread()
{
	thread = new boost::thread(&Joystick::run, this);
}

void Joystick::joinThread()
{
	// block till done
	if (thread) {
		thread->join();
	}
}

void Joystick::setVectors(const std::vector<unsigned char>& byteArray_,
		const std::vector<JoyAxis>& axes_, const std::vector<JoyButton>& buttons_)
{
	config.setVectors(byteArray_, axes_, buttons_);

	config.setModifiers(options->tButtonRemapping->getValue(),
			options->tAxesRemapping->getValue(), options->tAxisMultiplier->getValue());

}

std::string Joystick::getName() const
{
	return name;
}

void Joystick::setName(const std::string& name_)
{
	name = name_;
}

void Joystick::run()
{
	hid_set_nonblocking(handle,1);
	unsigned char buffer[256];
	int res;
	sensor_msgs::Joy joyMsg;
	joyMsg.header.frame_id = frameID;
	while(ros::ok())
	{

		if ((res = hid_read(handle, buffer, sizeof(buffer))) > 0) {

			joyMsg.header.stamp = ros::Time::now();
			// store values
			std::vector<float> saveAxes = joyMsg.axes;
			std::vector<int> saveButtons = joyMsg.buttons;

			// Fill new.
			config.parseInput(buffer, joyMsg);

			bool axesUnchanged = std::equal(saveAxes.begin(), saveAxes.end(), joyMsg.axes.begin());
			bool buttonsUnchanged = std::equal(saveButtons.begin(), saveButtons.end(), joyMsg.buttons.begin());

			// apply remappings
			if (!(axesUnchanged && buttonsUnchanged)) {
				pub.publish(joyMsg);
			}

		}

		usleep(10);
	}
}

void Joystick::getJoysticks(const std::string& filename, std::vector<Joystick*>& joysticks)
{
	std::ifstream fin(filename.c_str());
	std::vector<YAML::Node> joystickConfigs = YAML::LoadAll(fin);

	std::string name;
	int vendorId;
	int productId;


	for (unsigned int i = 0; i < joystickConfigs.size(); ++i) {
		Joystick* joystick = NULL;
		try {
			name = joystickConfigs[i]["name"].as<std::string>();
			vendorId = joystickConfigs[i]["vendor-id"].as<int>();
			productId = joystickConfigs[i]["product-id"].as<int>();

			//ROS_INFO("Vendor: %d, Product: %d", vendorId, productId);

			joystick = new Joystick(vendorId, productId, name);
			if (joystick->isOpen()) {
				// Create and attach JoystickOptions
				const YAML::Node& byteNode = joystickConfigs[i]["defaultByte"];
				std::vector<unsigned char> byteArray(byteNode.size());
				for(unsigned j=0;j<byteNode.size();j++) {
					byteArray[j] = byteNode[j].as<int>();
				}

				const YAML::Node& axisNode = joystickConfigs[i]["axes"];
				std::vector<JoyAxis> axes(axisNode.size());
				for(unsigned j=0;j<axisNode.size();j++) {
					//std::cout << axisNode[j];
					axes[j] = axisNode[j].as<JoyAxis>();
				}

				const YAML::Node& buttonNode = joystickConfigs[i]["buttons"];
				std::vector<JoyButton> buttons(buttonNode.size());
				for(unsigned j=0;j<buttonNode.size();j++) {
					buttons[j] = buttonNode[j].as<JoyButton>();
				}

				// set config
				joystick->setVectors(byteArray, axes, buttons);

				joysticks.push_back(joystick);
			} else {
				delete joystick;
			}

		} catch (YAML::Exception &e) {
			ROS_ERROR_STREAM("YAML Exception: " << e.what());
			joysticks.clear();
		} catch (std::runtime_error &e) {
			ROS_ERROR_STREAM("YAML Exception: " << e.what());
			joysticks.clear();
		}
	}

	fin.close();


}

} /* namespace telekyb */
