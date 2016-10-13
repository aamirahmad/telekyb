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


/// \file IIRFilter.h
/// \author Antonio Franchi
/// \brief Provides a simple class for Infinite impulse response recursive filter.

///  \addtogroup filters_Module
/* @{ */

#ifndef IIR_FILTER_HPP_
#define IIR_FILTER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Time.hpp>

#include <vector>
#include <list>

namespace TELEKYB_NAMESPACE
{

// esempio per order = 3
// implementa la funzione di trasferimento:
// _inCoeff[0]  +  _inCoeff[1] z +  _inCoeff[2] z^2 + _inCoeff[3] z^3
// ------------------------------------------------------------------
//      _outCoeff[0] + _outCoeff[1] z + _outCoeff[2] z^2 + z^3
//
// ossia
//
//y[t] =  _inCoeff[0] * u[t-3] +  _inCoeff[1] * u[t-2] +  _inCoeff[2] * u[t-1] + _inCoeff[3] * u[t]
//		 - _outCoeff[0] * y[t-3] - _outCoeff[1] * y[t-2] - _outCoeff[2] * y[t-1]


class IIRLowPass {};
class IIRFiltDeriv {};
class IIRLowPassButtW {};
class IIRFiltDerivButtW {};

/// \class IIRFilter
/// \brief Implements a discrete filter.
/// \author Antonio Franchi
/// \class IIRFilter
/// \brief Implements a discrete filter.
/// \author Antonio Franchi
class IIRFilter{
	private:
		std::vector< std::vector<double> > _inCoeff;
		std::vector<double> _outCoeff;
		std::vector< std::list<double> > _pastIns;
		std::list<double> _pastOuts;
		unsigned int _order;
		unsigned int _inNum;
		std::list<double>::iterator outIt,inIt;
		unsigned int iCo,iIn;

		void _kernelConstructor(std::vector< std::vector<double> > &inputCoeff,std::vector<double> &outputCoeff);
		void _kernelConstructor(std::vector< std::vector<double> > &inputCoeff,std::vector<double> &outputCoeff, std::vector< std::list<double> > pastIns0, std::list<double> pastOuts0);
	public:

		/// \brief One input constructor.
		/// \param[in] inputCoeff Coefficients of the denominator of the Zeta transform. \n
		/// Ex. If you want b0 + b1*z + b2*z^2 +z^3 you have to set: \n
		///	inputCoeff[0] = b0, inputCoeff[1] = b1, inputCoeff[2] = b2 \n
		/// Note that the last coefficient is not involved since it is always considered as 1.
		/// \param[in] outputCoeff Coefficients of the numerator of the Zeta transform. \n
		/// Ex. If you want a0 + a1*z + a2*z^2 + a3*z^3 you have to set: \n
		///	outputCoeff[0] = a0, outputCoeff[1] = a1, outputCoeff[2] = a2, outputCoeff[3] = a3 \n
		/// Note that if outputCoeff.size() > inputCoeff.size() + 1 then outputCoeff is cut at the entry outputCoeff[inputCoeff.size()] (included).
		/// This is needed to have a causal filter.
		IIRFilter(std::vector<double> &inputCoeff,std::vector<double> &outputCoef);

		/// \brief Multi input constructor.
		/// \param[in] inputCoeff Vector of Coefficients of the denominators of the Zeta transform for each different input. \n
		/// Ex. If you want b0 + b1*z + b2*z^2 +z^3 you have to set: \n
		///	inputCoeff[0] = b0, inputCoeff[1] = b1, inputCoeff[2] = b2 \n
		/// Note that the last coefficient is not involved since it is always considered as 1.
		/// \param[in] outputCoeff Coefficients of the numerator of the Zeta transform. \n
		/// Ex. If you want a0 + a1*z + a2*z^2 + a3*z^3 you have to set: \n
		///	outputCoeff[0] = a0, outputCoeff[1] = a1, outputCoeff[2] = a2, outputCoeff[3] = a3 \n
		/// Note that if outputCoeff.size() > inputCoeff.size() + 1 then outputCoeff is cut at the entry outputCoeff[inputCoeff.size()] (included).
		/// This is needed to have a causal filter.
		IIRFilter(std::vector< std::vector<double> > &inputCoeff,std::vector<double> &outputCoef);

		/// \brief Low Pass Filter with sample time ts  : wn^2/(s^2 + 2*csi*wn*s + wn^2)
		/// \param[in] dummyType It indicates the type of filter.
		/// \param[in] wn Natural frequency [rad/s].
		/// \param[in] csi Damping ratio.
		/// \param[in] ts Sampling time [s].
		IIRFilter(IIRLowPass dummyType, double wn, double csi, double ts);
		IIRFilter(IIRLowPass dummyType, double wn, double csi, double ts, std::list<double> pastIns0, std::list<double> pastOuts0);

		/// \brief Filtered Derivative with sample time ts  : (wn^2*s)/(s^2 + 2*csi*wn*s + wn^2)
		/// \param[in] dummyType It indicates the type of filter.
		/// \param[in] wn Natural frequency [rad/s].
		/// \param[in] csi Damping ratio.
		/// \param[in] ts Sampling time [s].
		IIRFilter(IIRFiltDeriv dummyType, double wn, double csi, double ts);

		



		/// \brief 4-th order Butterworth Low Pass Filter with sample time ts  :
		/// \param[in] dummyType It indicates the type of filter.
		/// \param[in] wn Natural frequency [rad/s].
		/// \param[in] csi Damping ratio.
		/// \param[in] ts Sampling time [s].
		IIRFilter(IIRLowPassButtW dummyType, double wn, double ts);



		/// \brief Filtered Derivative with sample time ts  and 4-th order Butterworth Low Pass Filter:
		/// \param[in] dummyType It indicates the type of filter.
		/// \param[in] wn Natural frequency [rad/s].
		/// \param[in] csi Damping ratio.
		/// \param[in] ts Sampling time [s].
		IIRFilter(IIRFiltDerivButtW dummyType, double wn, double ts);




		~IIRFilter();

		/// \brief Copy constructor.
		IIRFilter(const IIRFilter& t);

		/// \brief Assigment operator =.
		IIRFilter& operator=(const IIRFilter& t);

		/// \brief Executes a step of the recursive filter.
		/// \note If the filter has more than one input the inputs other than the first are put to zero.
		/// \param[in] in Input value.
		/// \param[out] out Output value.
		void step(double in,double& out);

		/// \brief Executes a step of the recursive filter, in case of multiple inputs.
		/// \param[in] in Input values.
		/// \param[out] out Output value.
		void step(const std::vector<double>& in,double& out);
		
		// THIS METHOD IS ONLY USED IN PARTICULAR CASES!!! 
		void set_w(double w, double csi, double ts);
};



} // namespace

#endif

/* @} */


