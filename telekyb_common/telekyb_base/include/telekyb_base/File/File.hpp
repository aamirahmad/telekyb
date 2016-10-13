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
// Contact info: martin.riedel@tuebingen.mpg.de
//
// ----------------------------------------------------------------------------


/// \file File.h
/// \author Antonio Franchi
/// \brief a simple class to manage files
/// \todo almost all

/// \defgroup file_Module File
/// \brief basic set of classes about text file managment
/// \ingroup baselib_Comp

///  \addtogroup file_Module
/* @{ */

#ifndef FILE_HPP_
#define FILE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

// stl
#include <vector>
#include <string>
#include <fstream>


namespace TELEKYB_NAMESPACE
{

/// \class File
/// \brief a simple class to manage files
/// \author Antonio Franchi antonio.franchi@tuebingen.mpg.de
class File {
	private:
		std::string fileName;
		std::ifstream file;

	public:
		/// Constructor.
		/// \param name The file name.
		File(const std::string& fileName_, std::ifstream::openmode mode = std::ifstream::in);
		~File();

		/// Reads all words in the file.
		/// Quotes Words will be treated as one word.
		bool getWords(std::vector<std::string>& wordVector);
};


}


#endif

/* @} */



