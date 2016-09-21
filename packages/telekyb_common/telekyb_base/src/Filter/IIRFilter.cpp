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


#include <telekyb_base/Filter/IIRFilter.hpp>

#include <math.h>
#include <stdexcept>

namespace TELEKYB_NAMESPACE
{

void IIRFilter::_kernelConstructor(std::vector< std::vector<double> > &inputCoeff,std::vector<double> &outputCoeff){
	_inCoeff  = inputCoeff;
	_inNum    = _inCoeff.size();
	_outCoeff = outputCoeff;
	_order    = _outCoeff.size();
	//Filter must be causal, future output will be dropped.
	for(unsigned int i=0;i<_inNum;i++){
		_inCoeff[i].resize(_order + 1,0.0); // size is 1 more for the coeficient of the current input
	}

	_pastIns.resize(_inNum);
	for(unsigned int i=0;i<_inNum;i++){
		_pastIns[i].resize(_order,0.0);
	}
	_pastOuts.resize(_order,0.0);
}



void IIRFilter::_kernelConstructor(std::vector< std::vector<double> > &inputCoeff,std::vector<double> &outputCoeff, std::vector< std::list<double> > pastIns0, std::list<double> pastOuts0){
	_inCoeff  = inputCoeff;
	_inNum    = _inCoeff.size();
	_outCoeff = outputCoeff;
	_order    = _outCoeff.size();
	//Filter must be causal, future output will be dropped.
	for(unsigned int i=0;i<_inNum;i++){
		_inCoeff[i].resize(_order + 1,0.0); // size is 1 more for the coeficient of the current input
	}


	if(pastIns0.size()!=_inNum){
		throw std::exception();
	}

	for(unsigned int i=0;i<_inNum;i++){
		if(pastIns0[i].size()!= _order){
			throw std::exception();
		}
	}

	if(pastOuts0.size()!=_order){
		throw std::exception();
	}

	_pastIns = pastIns0;
	_pastOuts = pastOuts0;
}




IIRFilter::IIRFilter(std::vector<double> &inputCoeff,std::vector<double> &outputCoeff){
	std::vector< std::vector<double> > iCs(1,inputCoeff);
	_kernelConstructor(iCs,outputCoeff);
}

IIRFilter::IIRFilter(std::vector< std::vector<double> > &inputCoeff,std::vector<double> &outputCoef){
	_kernelConstructor(inputCoeff,outputCoef);
}

IIRFilter::IIRFilter(const IIRFilter& t) {
	_inCoeff  = t._inCoeff;
	_outCoeff = t._outCoeff;
	_pastIns  = t._pastIns;
	_pastOuts = t._pastOuts;
	_order    = t._order;
	_inNum		= t._inNum;
}

IIRFilter& IIRFilter::operator=(const IIRFilter& t){
	if (this != &t){
		_inCoeff  = t._inCoeff;
		_outCoeff = t._outCoeff;
		_pastIns  = t._pastIns;
		_pastOuts = t._pastOuts;
		_order    = t._order;
		_inNum		= t._inNum;
	}
	return *this;
}

IIRFilter::IIRFilter(IIRLowPass dummyType, double wn, double csi, double ts){

	std::vector<double> inputCoeff(3, 0.0), outputCoeff(2, 0.0);

	inputCoeff[0]=wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	inputCoeff[1]=2.0*wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	inputCoeff[2]=wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

	outputCoeff[0]=(-4.0*csi*wn*ts+4+wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	outputCoeff[1]=(-8.0+2.0*wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

	std::vector< std::vector<double> > iCs(1,inputCoeff);

	_kernelConstructor(iCs, outputCoeff);

}

void IIRFilter::set_w(double wn, double csi, double ts) 
{	
  std::vector<double> inputCoeff(3, 0.0), outputCoeff(2, 0.0);

  inputCoeff[0]=wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
  inputCoeff[1]=2.0*wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
  inputCoeff[2]=wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

  outputCoeff[0]=(-4.0*csi*wn*ts+4+wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
  outputCoeff[1]=(-8.0+2.0*wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
  
  std::vector< std::vector<double> > iCs(1,inputCoeff);
  _inCoeff = iCs;
  _outCoeff = outputCoeff;
//   std::vector< std::vector<double> > iCs(1,inputCoeff);
// 
//   _kernelConstructor(iCs, outputCoeff);
}


IIRFilter::IIRFilter(IIRLowPass dummyType, double wn, double csi, double ts, std::list<double> pastIns0, std::list<double> pastOuts0){
	std::vector<double> inputCoeff(3, 0.0), outputCoeff(2, 0.0);

	inputCoeff[0]=wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	inputCoeff[1]=2.0*wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	inputCoeff[2]=wn*wn*ts*ts/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

	outputCoeff[0]=(-4.0*csi*wn*ts+4+wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	outputCoeff[1]=(-8.0+2.0*wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

	std::vector< std::vector<double> > iCs(1,inputCoeff);

	_kernelConstructor(iCs, outputCoeff, std::vector< std::list<double> >(1, pastIns0), pastOuts0);

}




IIRFilter::IIRFilter(IIRFiltDeriv dummyType, double wn, double csi, double ts){

	std::vector<double> inputCoeff(3, 0.0), outputCoeff(2, 0.0);

	inputCoeff[0]=-2.0*ts*wn*wn/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	inputCoeff[1]=0.0;
	inputCoeff[2]=2.0*ts*wn*wn/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

	outputCoeff[0]=(-4.0*csi*wn*ts+4+wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);
	outputCoeff[1]=(-8.0+2.0*wn*wn*ts*ts)/(4.0+wn*wn*ts*ts+4.0*csi*wn*ts);

	std::vector< std::vector<double> > iCs(1,inputCoeff);

	_kernelConstructor(iCs, outputCoeff);

}





IIRFilter::IIRFilter(IIRLowPassButtW dummyType, double wn, double ts){

	std::vector<double> inputCoeff(5, 0.0), outputCoeff(4, 0.0);

	double a=1.0000;
	double b=2.61312592975275;
	double c=3.41421356237309;
	double d=2.61312592975275;
	double e=1.0000;

	double scale=e*pow(ts,4)*pow(wn,4) + 2*d*pow(ts,3)*pow(wn,3) + 4*c*pow(ts,2)*pow(wn,2) + 8*b*ts*wn + 16*a;

	inputCoeff[0]=(pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[1]=(4*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[2]=(6*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[3]=(4*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[4]=(pow(ts,4)*pow(wn,4))/scale;


	outputCoeff[0]=(e*pow(ts,4)*pow(wn,4) - 2*d*pow(ts,3)*pow(wn,3) + 4*c*pow(ts,2)*pow(wn,2) - 8*b*ts*wn + 16*a)/scale;
	outputCoeff[1]=(4*e*pow(ts,4)*pow(wn,4) - 4*d*pow(ts,3)*pow(wn,3) + 16*b*ts*wn - 64*a)/scale;
	outputCoeff[2]=(6*e*pow(ts,4)*pow(wn,4) - 8*c*pow(ts,2)*pow(wn,2) + 96*a)/scale;
	outputCoeff[3]=(4*e*pow(ts,4)*pow(wn,4) + 4*d*pow(ts,3)*pow(wn,3) - 16*b*ts*wn - 64*a)/scale;


/* MATLAB CODE
a=1.0000;
b=2.61312592975275;
c=3.41421356237309;
d=2.61312592975275;
e=1.0000;


numcoeff(1)=T^4*wn^4;
numcoeff(2)=4*T^4*wn^4;
numcoeff(3)=6*T^4*wn^4;
numcoeff(4)=4*T^4*wn^4;
numcoeff(5)=T^4*wn^4;

dencoeff(1)=e*T^4*wn^4 + 2*d*T^3*wn^3 + 4*c*T^2*wn^2 + 8*b*T*wn + 16*a;
dencoeff(2)=4*e*T^4*wn^4 + 4*d*T^3*wn^3 - 16*b*T*wn - 64*a;
dencoeff(3)=6*e*T^4*wn^4 - 8*c*T^2*wn^2 + 96*a;
dencoeff(4)=4*e*T^4*wn^4 - 4*d*T^3*wn^3 + 16*b*T*wn - 64*a;
dencoeff(5)=e*T^4*wn^4 - 2*d*T^3*wn^3 + 4*c*T^2*wn^2 - 8*b*T*wn + 16*a;



*/




	std::vector< std::vector<double> > iCs(1,inputCoeff);

	_kernelConstructor(iCs, outputCoeff);

}



IIRFilter::IIRFilter(IIRFiltDerivButtW dummyType, double wn, double ts){

	std::vector<double> inputCoeff(6, 0.0), outputCoeff(5, 0.0);

	double a=1.0000;
	double b=2.61312592975275;
	double c=3.41421356237309;
	double d=2.61312592975275;
	double e=1.0000;

	double scale=ts*(e*pow(ts,4)*pow(wn,4) + 2*d*pow(ts,3)*pow(wn,3) + 4*c*pow(ts,2)*pow(wn,2) + 8*b*ts*wn + 16*a);

	inputCoeff[0]=(-2*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[1]=(-6*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[2]=(-4*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[3]=(4*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[4]=(6*pow(ts,4)*pow(wn,4))/scale;
	inputCoeff[5]=(2*pow(ts,4)*pow(wn,4))/scale;


	outputCoeff[0]=(ts*(e*pow(ts,4)*pow(wn,4) - 2*d*pow(ts,3)*pow(wn,3) + 4*c*pow(ts,2)*pow(wn,2) - 8*b*ts*wn + 16*a))/scale;
	outputCoeff[1]=(ts*(5*e*pow(ts,4)*pow(wn,4) - 6*d*pow(ts,3)*pow(wn,3) + 4*c*pow(ts,2)*pow(wn,2) + 8*b*ts*wn - 48*a))/scale;
	outputCoeff[2]=(ts*(10*e*pow(ts,4)*pow(wn,4) - 4*d*pow(ts,3)*pow(wn,3) - 8*c*pow(ts,2)*pow(wn,2) + 16*b*ts*wn + 32*a))/scale;
	outputCoeff[3]=(ts*(10*e*pow(ts,4)*pow(wn,4) + 4*d*pow(ts,3)*pow(wn,3) - 8*c*pow(ts,2)*pow(wn,2) - 16*b*ts*wn + 32*a))/scale;
	outputCoeff[4]=(ts*(5*e*pow(ts,4)*pow(wn,4) + 6*d*pow(ts,3)*pow(wn,3) + 4*c*pow(ts,2)*pow(wn,2) - 8*b*ts*wn - 48*a))/scale;


/* MATLAB CODE
a=1.0000;
b=2.61312592975275;
c=3.41421356237309;
d=2.61312592975275;
e=1.0000;


numcoeff(1)=2*ts^4*wn^4;
numcoeff(2)=6*ts^4*wn^4;
numcoeff(3)=4*ts^4*wn^4;
numcoeff(4)=-4*ts^4*wn^4;
numcoeff(5)=-6*ts^4*wn^4;
numcoeff(6)=-2*ts^4*wn^4;




dencoeff(1)=ts*(e*ts^4*wn^4 + 2*d*ts^3*wn^3 + 4*c*ts^2*wn^2 + 8*b*ts*wn + 16*a);
dencoeff(2)=ts*(5*e*ts^4*wn^4 + 6*d*ts^3*wn^3 + 4*c*ts^2*wn^2 - 8*b*ts*wn - 48*a);
dencoeff(3)=ts*(10*e*ts^4*wn^4 + 4*d*ts^3*wn^3 - 8*c*ts^2*wn^2 - 16*b*ts*wn + 32*a);
dencoeff(4)=ts*(10*e*ts^4*wn^4 - 4*d*ts^3*wn^3 - 8*c*ts^2*wn^2 + 16*b*ts*wn + 32*a);
dencoeff(5)=ts*(5*e*ts^4*wn^4 - 6*d*ts^3*wn^3 + 4*c*ts^2*wn^2 + 8*b*ts*wn - 48*a);
dencoeff(6)=ts*(e*ts^4*wn^4 - 2*d*ts^3*wn^3 + 4*c*ts^2*wn^2 - 8*b*ts*wn + 16*a);


*/




	std::vector< std::vector<double> > iCs(1,inputCoeff);

	_kernelConstructor(iCs, outputCoeff);

}







IIRFilter::~IIRFilter(){
}


void IIRFilter::step(double in,double& out){
//   std::cout << " _inNum  " << _inNum << " in " << in << std::endl;
	std::vector<double> inputs(_inNum,0.0);
// 	inputs[1] = in;
	inputs[0] = in;
	step(inputs,out);
}

void IIRFilter::step(const std::vector<double> &in,double &out){
	out = 0.0;
	for(iCo=0, outIt=_pastOuts.begin() ;iCo<_order ; iCo++, outIt++){
		out += -(*outIt)*_outCoeff[iCo];
//   std::cout << " iCo  " << iCo << " outCoeff[iCo] " << _outCoeff[iCo] << std::endl;
	}
	for(iIn=0;iIn<_inNum;iIn++){
		for(iCo=0,inIt=_pastIns[iIn].begin();iCo<_order;iCo++,inIt++){
			out += (*inIt)*_inCoeff[iIn][iCo] ;
		}
		out += in[iIn] * _inCoeff[iIn][_order];
//   std::cout << " iIn  " << iIn << " in[iIn] " << in[iIn] << std::endl;
	}
	for(iIn=0;iIn<_inNum;iIn++){
		_pastIns[iIn].pop_front();
		_pastIns[iIn].push_back(in[iIn]);
	}
	_pastOuts.pop_front();
	_pastOuts.push_back(out);
}

} // namespace
