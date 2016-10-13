// ----------------------------------------------------------------------------
//
// $Id$
//
// Copyright 2008, 2009, 2010, 2011  Antonio Franchi
//
// This file is part of TeleKyb.
//
// TeleKyb is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TeleKyb is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TeleKyb. If not, see <http://www.gnu.org/licenses/>.
//
// Contact info: antonio.franchi@tuebingen.mpg.de 
//
// ----------------------------------------------------------------------------


/// \file FirFilter.h
/// \author Antonio Franchi
/// \brief Provides a simple class for Finite impulse response filter.

/// \defgroup filters_Module Recursive Filters
/// \brief This module provides simple classes for recursive filters.
/// \ingroup algorithms_Comp


///  \addtogroup filters_Module
/* @{ */

#ifndef FIR_FILTER_HPP_
#define FIR_FILTER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Time.hpp>

#include <list>

#define FIR_TIME_DEBUG 0

namespace TELEKYB_NAMESPACE
{


enum FirFilterShapes{
	FIR_SHAPES_CONSTANT,
	FIR_SHAPES_NUM
};

struct FirFilterPar{
	FirFilterShapes shape;
	Time lag;
	
	FirFilterPar(FirFilterShapes s=FIR_SHAPES_CONSTANT, Time l=Time(1,0)){
		shape = s;
		lag = l;
	}
};

struct FirFilterVar{
	std::list<double> samples;
	std::list<Time> timeStamps;
	Time lastTimeStamp;
};

struct FirFilterIn{
	double sample;
	
	FirFilterIn(double s=0.0){
		sample = s;
	}
};

struct FirFilterOut{
	double sample;
	int size;
};

class FirFilter{
	private:
		FirFilterPar _par;
		FirFilterVar _var;
		
		void _eraseOldSamples();
		void _average(FirFilterOut& out);
		
	public:
		FirFilter(FirFilterPar par){
			_par = par;
		}
		
		void step(FirFilterIn& in,FirFilterOut& out);
};

#endif

} // namespace
/* @} */


