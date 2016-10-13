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


#include <telekyb_base/Filter/OneEuroFilter.hpp>

#include <math.h>
#include <stdexcept>

namespace TELEKYB_NAMESPACE
{
  
  
  
  
  	// ################################################################
	// #####   Methods of class HighPassFilter   #######################
	// ################################################################
  void HighPassFilter::setAlpha(double alpha) {
    if (alpha<=0.0 || alpha>1.0)
      throw std::range_error("alpha should be in (0.0., 1.0]") ;
    a = alpha ;
  }

  HighPassFilter::HighPassFilter(double alpha, double initval) {
    y = s = initval ;
    setAlpha(alpha) ;
    initialized = false ;
  }

  double HighPassFilter::filter(double value) {
    double result ;
    if (initialized)
      result = a*s +  a*(value - y) ;
    else {
      result = value ;
      initialized = true ;
    }
    y = value ;
    s = result ;
    return result ;
  }

  double HighPassFilter::filterWithAlpha(double value, double alpha) {
    setAlpha(alpha) ;
    return filter(value) ;
  }

  bool HighPassFilter::hasLastRawValue(void) {
    return initialized ;
  }

  double HighPassFilter::lastRawValue(void) {
    return y ;
  }
  
  
  
  
  
  

	
	// ################################################################
	// #####   Methods of class LowPassFilter   #######################
	// ################################################################
  void LowPassFilter::setAlpha(double alpha) {
    if (alpha<=0.0 || alpha>1.0) {
	throw std::range_error("alpha should be in (0.0., 1.0]") ;
	    
    }
    a = alpha ;
  }

  LowPassFilter::LowPassFilter(double alpha, double initval) {
    y = s = initval ;
    setAlpha(alpha) ;
    initialized = false ;
  }

  double LowPassFilter::filter(double value) {
    double result ;
    if (initialized)
      result = a*value + (1.0-a)*s ;
    else {
      result = value ;
      initialized = true ;
    }
    y = value ;
    s = result ;
    return result ;
  }

  double LowPassFilter::filterWithAlpha(double value, double alpha) {
    setAlpha(alpha) ;
    return filter(value) ;
  }

  bool LowPassFilter::hasLastRawValue(void) {
    return initialized ;
  }

  double LowPassFilter::lastRawValue(void) {
    return y ;
  }
  
  
  
  
  
  
	// ################################################################
	// #####   Methods of class LowPassFilter   #######################
	// ################################################################

  
  
double OneEuroFilter::alpha(double cutoff) {
    double te = 1.0 / freq ;
    double tau = 1.0 / (2*M_PI*cutoff) ;
    return 1.0 / (1.0 + tau/te) ;
  }

  void OneEuroFilter::setFrequency(double f) {
    if (f<=0) throw std::range_error("freq should be >0") ;
    freq = f ;
  }

  void OneEuroFilter::setMinCutoff(double mc) {
    if (mc<=0) throw std::range_error("mincutoff should be >0") ;
    mincutoff = mc ;
  }

  void OneEuroFilter::setBeta(double b) {
    beta_ = b ;
  }

  void OneEuroFilter::setDerivateCutoff(double dc) {
    if (dc<=0) throw std::range_error("dcutoff should be >0") ;
    dcutoff = dc ;
  }

  OneEuroFilter::OneEuroFilter(double freq, 
		double mincutoff, double beta_, double dcutoff) {
    setFrequency(freq) ;
    setMinCutoff(mincutoff) ;
    setBeta(beta_) ;
    setDerivateCutoff(dcutoff) ;
    x = new LowPassFilter(alpha(mincutoff)) ;
    dx = new LowPassFilter(alpha(dcutoff)) ;
    lasttime = ONEEURO_FILTER_UNDEEFINED_TIME;
  }

  double OneEuroFilter::filter(double value, double timestamp) {
    // update the sampling frequency based on timestamps
    if (lasttime!=ONEEURO_FILTER_UNDEEFINED_TIME && timestamp!=ONEEURO_FILTER_UNDEEFINED_TIME)
      freq = 1.0 / (timestamp-lasttime) ;
    lasttime = timestamp ;
    // estimate the current variation per second 
    double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
    double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff)) ;
    // use it to update the cutoff frequency
    double cutoff = mincutoff + beta_*fabs(edvalue) ;
    // filter the given value
    return x->filterWithAlpha(value, alpha(cutoff)) ;
  }

  OneEuroFilter::~OneEuroFilter(void) {
    delete x ;
    delete dx ;
  }


} // namespace
