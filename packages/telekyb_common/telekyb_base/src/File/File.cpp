// ----------------------------------------------------------------------------
//
// $Id$
//
// Copyright 2008, 2009, 2010, 2011  Antonio Franchi and Paolo Stegagno    
//
// This file is part of MIP.
//
// MIP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MIP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MIP. If not, see <http://www.gnu.org/licenses/>.
//
// Contact info: antonio.franchi@tuebingen.mpg.de stegagno@dis.uniroma1.it
//
// ----------------------------------------------------------------------------


#include <telekyb_base/File/File.hpp>


//#include <Tools/TokenReader.h>
// stl
//#include <iostream>


// boost
#include <boost/tokenizer.hpp>

// ros
#include <ros/console.h>

// defines
#define FILE_COMMENT_CHAR '#'

namespace TELEKYB_NAMESPACE
{

typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;

File::File(const std::string& fileName_, std::ifstream::openmode mode)
{
	fileName = fileName_;
	file.open(fileName.c_str(), mode);

	if (!file.is_open())
	{
		ROS_WARN_STREAM(fileName << " does not exist!");
	}
}

File::~File()
{
	file.close();
}

bool File::getWords(std::vector<std::string>& wordVector)
{
	// exit conditions
	if (!file.is_open()) {
		ROS_WARN_STREAM("Cound not get Words from File! " << fileName << " does not exist!");
		return false;
	}

	// a line is always a word sparator
	std::string line;
	while(std::getline(file, line)) {
		// check if line has #. ignore rest
		size_t cPos = line.find( FILE_COMMENT_CHAR );
		line = line.substr(0,cPos);

		boost::escaped_list_separator<char> sep('\\',' ','\"');
		Tokenizer tokenizer(line,sep);

		for(Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); it++) {
			wordVector.push_back(*it);
		}
	}

	return true;
}

} // namespace


