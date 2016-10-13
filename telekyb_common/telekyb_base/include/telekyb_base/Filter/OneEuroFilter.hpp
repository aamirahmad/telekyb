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


/// \file OneEuroFilter.h
/// \author Antonio Franchi
/// \brief Provides a simple class for Infinite impulse response recursive filter.

///  \addtogroup filters_Module
/* @{ */

#ifndef ONEEURO_FILTER_HPP_
#define ONEEURO_FILTER_HPP_



#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>


#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Time.hpp>

#include <vector>
#include <list>

namespace TELEKYB_NAMESPACE
{

/* -*- coding: utf-8 -*-
 *
 * OneEuroFilter.cc -
 *
 * Author: Nicolas Roussel (nicolas.roussel@inria.fr)
 *
 */

// -----------------------------------------------------------------
// Utilities



#define ONEEURO_FILTER_UNDEEFINED_TIME -1.0





class HighPassFilter {
    
  double y, a, s;
  bool initialized ;

  void setAlpha(double alpha);

public:

  HighPassFilter(double alpha, double initval=0.0);

  double filter(double value);

  double filterWithAlpha(double value, double alpha);

  bool hasLastRawValue(void);

  double lastRawValue(void);

} ;





class LowPassFilter {
    
  double y, a, s ;
  bool initialized ;

  void setAlpha(double alpha);

public:

  LowPassFilter(double alpha, double initval=0.0);

  double filter(double value);

  double filterWithAlpha(double value, double alpha);

  bool hasLastRawValue(void);

  double lastRawValue(void);

} ;

// -----------------------------------------------------------------

class OneEuroFilter {

  double freq ;
  double mincutoff ;
  double beta_ ;
  double dcutoff ;
  LowPassFilter *x ;
  LowPassFilter *dx ;
  double lasttime ;

  double alpha(double cutoff);

  void setFrequency(double f);

  void setMinCutoff(double mc);

  void setBeta(double b);

  void setDerivateCutoff(double dc);

public:

  OneEuroFilter(double freq, double mincutoff=1.0, double beta_=0.0, double dcutoff=1.0);

  double filter(double value, double timestamp=ONEEURO_FILTER_UNDEEFINED_TIME);

  ~OneEuroFilter(void);

} ;




} // namespace

#endif

/* @} */


