#include <tk_be_common/EmergencyEscape/state.hpp>
using namespace std;

/*
* See: http://en.wikipedia.org/wiki/Stdarg.h
*/
double magnitude(const int &n, ...)
{
    va_list arglist;
    int j;
    double sum = 0;
 
    va_start(arglist, n); /* Requires the last fixed parameter (to get the address) */
    for (j = 0; j < n; j++) {
        sum += pow(va_arg(arglist, double),2); /* Increments ap to the next argument. */
    }
    va_end(arglist);
 
    return sqrt(sum);
}

/*
* Function to convert measure data (of type Measure) to synthetic state
*/
State MeasureToState(const Measure &m)
{
	State s;

	float rh  = m.r * cos(m.iv);  
	s.dch = rh * sin(m.ih);

	float dh = rh * cos(m.ih); 
	float Vh = m.V * cos(m.gamma);

	float dcv_iv   = rh * tan(m.iv); 
	float dcv_gamma = dh * tan(m.gamma); 
	s.dcv = dcv_gamma - dcv_iv; 

	float d = magnitude(2,abs(dh),dcv_gamma);
	if (dh < 0)
		d = d*(-1);

	s.tc = d/m.V;
	//s.tc = dh/Vh;

	return s;
}

/*
Convert world data (from obstacle reference frame) to measure data.
This might not be needed depending on the output of the sensors.
*/
Measure WorldToMeasure(const std::vector<float> &w)
{
	Measure m;

	m.r  = magnitude(3,w[0],w[1],w[2]);

	m.ih = atan2(w[4],w[3])-atan2(-w[1],-w[0]);

	if (m.ih > M_PI)
    	m.ih = m.ih - 2*M_PI;

	m.V 	= magnitude(3,w[3],w[4],w[5]);
	m.iv 	= asin(-w[2]/m.r);
	m.gamma = asin(w[5]/m.V);

	return m;

}

/*
* Convert measure to world in an obstacle frame of reference
* Assume y = 0 at all instances, thanks to symmetry
*/
vector<float> MeasureToWorld(const Measure &m)
{
	vector<float> w(6);

	w[0] = - m.r * cos(m.iv);
	w[1] = 0;
	w[2] = m.r * sin(m.iv);

	w[3] = m.V * cos(m.gamma) * cos(m.ih);
	w[4] = m.V * cos(m.gamma) * sin(m.ih);
	w[5] = m.V * sin(m.gamma);

	return w;
}
