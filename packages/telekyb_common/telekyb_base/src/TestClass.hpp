/*
 * TestClass.h
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#ifndef TESTCLASS_H_
#define TESTCLASS_H_


#include <telekyb_base/Options.hpp>
#include "Eigen/Dense"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <telekyb_base/Spaces.hpp>

//ENUM
#include <telekyb_defines/enum.hpp>

TELEKYB_ENUM(Level,
    (Abort)
    (Error)
    (Alert)
    (Info)
    (Trace)
    (Debug)
)

TELEKYB_ENUM_VALUES(LevelVal, const char*,
	(Abort)("unrecoverable problem")
	(Error)("recoverable problem")
	(Alert)("unexpected behavior")
	(Info) ("expected behavior")
	(Trace)("normal flow of execution")
	(Debug)("detailed object state listings")
)

class TestOptionContainer : public telekyb::OptionContainer {
public:
	TestOptionContainer();
	telekyb::Option<int>* intOp1;
	telekyb::Option<int>* intOp2;
	telekyb::Option<double>* doubleOp1;
	telekyb::Option<Eigen::Matrix3i>* matOp3;
	telekyb::Option<LevelValBaseEnum<const char*>::Type>* levelVarOption;
	telekyb::Option<LevelBaseEnum::Type >* levelOption;
	telekyb::Option<telekyb::Position3D>* pos3D;
	telekyb::Option<Eigen::Quaterniond>* testQuatOption;
	telekyb::Option<Eigen::Matrix<double, Eigen::Dynamic, 2> >* dynamicVec;
};

class TestListener : public telekyb::OptionListener<int>, public telekyb::OptionListener<double> {
	void optionDidChange(const telekyb::Option<int>* option);
	void optionShouldDelete(const telekyb::Option<int>* option);
	void optionDidChange(const telekyb::Option<double>* option) {
		std::cout << "Option changed! " << option->getValue() << std::endl;
	}
	void optionShouldDelete(const telekyb::Option<double>* option) {
		std::cout << "Option should deleted! " << option->getValue() << std::endl;
	}


};



#endif /* TESTCLASS_H_ */
