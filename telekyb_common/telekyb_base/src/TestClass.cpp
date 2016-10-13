/*
 * TestClass.cpp
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#include "TestClass.hpp"

#include <ros/ros.h>

#include <telekyb_base/Spaces.hpp>
#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_base/Time.hpp>

#include <iostream>

#include <telekyb_base/ROS/GenericSubscriber.hpp>
#include <std_msgs/Float32.h>


using namespace TELEKYB_NAMESPACE;

TestOptionContainer::TestOptionContainer()
	: OptionContainer("TestOptionContainer")
{
    intOp1 = OptionContainer::addOption<int>("int1","Bla bla", 10);
    intOp2 = OptionContainer::addBoundsOption<int>("int2","Bla bla", 20, 50, 5, true, true);
    doubleOp1 = OptionContainer::addBoundsOption<double>("double", "BlaBla", 3.5, 2.0, 10.3);
	Eigen::Matrix3i defaultVal = Eigen::Matrix3i::Zero();
	defaultVal.diagonal() = Eigen::Vector3i(1,2,3);
    matOp3 = OptionContainer::addOption<Eigen::Matrix3i>("matrix","test", defaultVal, false);
    levelVarOption = OptionContainer::addOption< LevelValBaseEnum< const char * >::Type >("enum23","That's an enum", LevelVal::Debug);
    levelOption = OptionContainer::addOption< LevelBaseEnum::Type >("enum","That's an enum", Level::Debug, false);
    pos3D = OptionContainer::addOption<telekyb::Position3D>("pos3D","test", telekyb::Position3D(), false);
	testQuatOption = addOption<Eigen::Quaterniond>("testQuatOption","test", Eigen::Quaterniond(), false);
	Eigen::Matrix<double, Eigen::Dynamic, 2> testVal(4,2);
    dynamicVec = OptionContainer::addOption<Eigen::Matrix<double, Eigen::Dynamic, 2> >(
			"dynamicVec",
			"test of Dynamic Vector",
			testVal, false);
}

void TestListener::optionDidChange(const telekyb::Option<int>* option) {
	std::cout << "Option changed! " << option->getValue() << std::endl;
}

void TestListener::optionShouldDelete(const telekyb::Option<int>* option) {
	std::cout << "Option about to be deleted! " << option->getValue() << std::endl;
}


// main
int main(int argc, char* argv[])
{
//	telekyb::GenericSubscriber<std_msgs::Float32> sub(ros::NodeHandle(), "test", 10);
//	std_msgs::Float32 test = sub.getLastMsg();
	//ROSCONSOLE_AUTOINIT;

	//log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

	// Set the logger for this package to output all statements
	//my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);



//	Level lev = Level::Abort;
//	std::cout << "Index: " << lev.index() << "String: " << lev.str() << "." << std::endl;
//	if(Level::get_by_name("Info")) {
//		std::cout << "Level knows variable" << std::endl;
//		lev = *Level::get_by_name("Info");
//		//const char* test = Level::names();
//	} else {
//		std::cout << "Level does not know variable" << std::endl;
//	}
//	std::cout << "Index: " << lev.index() << " String: " << lev.str() << "." << std::endl;
//	YAML::Node testNode = YAML::Load("0");
//	bool testValue = testNode.as<bool>();
//
//	if (testValue) {
//		ROS_INFO("YAML produced true");
//	} else {
//		ROS_INFO("YAML produced false");
//	}

	telekyb::TeleKyb::init(argc,argv, "TestClass");

	TestListener tl;
//	telekyb::OptionListener<int>::registerOptionListener(&tl);
//	telekyb::OptionListener<double>::registerOptionListener(&tl);

	TestOptionContainer *t = new TestOptionContainer();
	t->intOp1->setValue(200);
	std::cout << "Value: " << t->intOp1->getValue() << std::endl;
	std::cout << "Value: " << t->intOp2->getValue() << std::endl;

	t->intOp2->setValue(10);
	t->intOp2->setValue(30);
	t->intOp2->setValue(50);
	t->intOp2->setValue(5);

	t->intOp2->setValue(51);
	t->intOp2->setValue(4);

	t->doubleOp1->setValue(7.03);
	t->doubleOp1->setValue(10.3);
	t->doubleOp1->setValue(10.4);

//	Eigen::MatrixXd test1(3,3);
//	Eigen::VectorXd test2(3);
//	test2 << 1,2,3;
//	test2.resize(4);
//	std::cout << "Vector: " << test2 << std::endl;
//	ROS_INFO("Rows at compile time: %d, %d",test1.RowsAtCompileTime,1);
//	test1.resize(4,4);

	std::cout << "Dynamic Vector: " << std::endl << t->dynamicVec->getValue() << std::endl;


	//t->levelOption->setValue(Level::Alert);

	Level testLevel = t->levelOption->getValue();
	ROS_INFO_STREAM("Option: " << t->levelOption->getName() << " Value: " << testLevel.str());

//	if (static_cast<Level>() == Level::Alert) {
//		// awesome
//		std::cout << "worked!" << std::endl;
//	}

	std::cout << t->matOp3->getValue() << std::endl;

	std::cout << t->pos3D->getValue() << std::endl;


//	Level lev = t->levelOption->getValue();
//	std::cout << "Index: " << lev.m_index << "String: " << lev.str() << "." << std::endl();



//	// Time Stuff
//	telekyb::Time time;
//	ROS_INFO_STREAM("Current Time: " << time.dateTimeToString());
//
//	// Timer Test
//	telekyb::Timer timer;
//	sleep(3);
//	ROS_INFO_STREAM("Timer elapsed: " << timer.toString());
//
//	telekyb::Time test2(2.3);
//	telekyb::Timer timer2(test2);
//	sleep(1);
//	ROS_INFO_STREAM("Timer elapsed: " << timer2.toString());
//
//	telekyb::Duration dur(3,0);
//	telekyb::Timer timer3;
//	dur.sleep();
//	ROS_INFO_STREAM("Timer elapsed: " << timer3.toString());


	/*
	 * DONT TEST AFTER THIS POINT
	 */
	ros::spin();
//
	delete t;
//
//	Eigen::Matrix3d mat;
//
//	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> test;




	telekyb::TeleKyb::shutdown();

	return 0;
}
