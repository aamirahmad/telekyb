/*
 * BagFile.hpp
 *
 *  Created on: Apr 30, 2012
 *      Author: mriedel
 */

#ifndef BAGFILE_HPP_
#define BAGFILE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include "BagExportOptions.hpp"

namespace TELEKYB_NAMESPACE
{

class BagFile {
private:
	BagExportOptions options;

public:
	BagFile();
	virtual ~BagFile();

	void process();
	void processMat();
};

}

#endif /* BAGFILE_HPP_ */
