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


#include <telekyb_base/Filter/FirFilter.hpp>

#include <ros/console.h>

namespace TELEKYB_NAMESPACE
{

void FirFilter::_eraseOldSamples(){
	std::list<double>::iterator sampIt = _var.samples.begin();
	std::list<Time>::iterator    timeIt = _var.timeStamps.begin();
	std::list<double>::iterator sampItEnd = _var.samples.end();
	sampItEnd--;
	std::list<Time>::iterator    timeItEnd = _var.timeStamps.end();
	timeItEnd--;
	while((sampIt != (sampItEnd)) && (timeIt != (timeItEnd))){
		if(*timeIt >= _var.lastTimeStamp - _par.lag){
			break;
		}
		sampIt++;
		timeIt++; 
	}
	if(timeIt != _var.timeStamps.begin()){
		_var.samples.erase(_var.samples.begin(),sampIt);
		_var.timeStamps.erase(_var.timeStamps.begin(),timeIt);
	}
}
		
void FirFilter::_average(FirFilterOut& out){
	double sum = 0.0;
	std::list<double>::iterator sampIt = _var.samples.begin();
	while(sampIt != _var.samples.end()){
		sum += *sampIt;
		sampIt++;
	}
	out.size = _var.samples.size();
	if(out.size == 0){
		ROS_WARN("FirFilter: Number of samples zero, something's wrong.");
	}
	out.sample = sum /(double) out.size;
}

void  FirFilter::step(FirFilterIn& in,FirFilterOut& out){
	_var.samples.push_back(in.sample);
	_var.lastTimeStamp=Time();
	_var.timeStamps.push_back(_var.lastTimeStamp);
	_eraseOldSamples();
	_average(out);
}

} // namespace

