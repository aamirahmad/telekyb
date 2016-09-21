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




using namespace TELEKYB_NAMESPACE;

// -----------------------------------------------------------------

int main(int argc, char **argv) {
  srand(time(0));

  double duration = 10.0 ; // seconds

  double frequency = 120 ; // Hz
  double mincutoff = 1.0 ; // FIXME
  double beta = 1.0 ;      // FIXME
  double dcutoff = 1.0 ;   // this one should be ok
  double dcutoff1 = 0.1 ;   // this one should be ok

//   std::cout << "#SRC OneEuroFilter.cc" << std::endl
// 	    << "#CFG {'beta': " << beta << ", 'freq': " << frequency << ", 'dcutoff': " << dcutoff << ", 'mincutoff': " << mincutoff << "}" << std::endl
// 	    << "#LOG timestamp, signal, noisy, filtered" << std::endl ;

  OneEuroFilter f(frequency, mincutoff, beta, dcutoff) ;
  LowPassFilter f1(dcutoff1, 0.0) ;
  for (double timestamp=0.0; timestamp<duration; timestamp+=1.0/frequency) {
    double signal = sin(timestamp) ;
    double noisy = signal + (rand()/double(RAND_MAX)-0.5)/5.0 ;
    double filtered = f.filter(noisy, timestamp) ;
    double filtered1 = f1.filter(noisy) ;
    std::cout << timestamp << ", "
	      << signal << ", "
	      << noisy << ", "
	      << filtered << ", "
	      << filtered1
	      << std::endl ;
  }
  
  return 0 ;
}