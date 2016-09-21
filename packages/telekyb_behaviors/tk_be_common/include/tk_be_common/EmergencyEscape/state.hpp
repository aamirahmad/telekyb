/* The following file holds a set of common definitions */
#ifndef STATE_H
#define STATE_H

#include <cmath>
#include <algorithm>  //std::min
#include <vector>
#include <iostream>

#include "stdarg.h"
using namespace std;

/* Structure to hold the relevant state information */
typedef struct
{
   float  dch, dcv, tc;
}State;

/* Structure to hold the relevant measurement information */
typedef struct
{
   float  r, ih, V, iv, gamma;
}Measure;

double magnitude(const int &n, ...);
State MeasureToState(const Measure &m);
Measure WorldToMeasure(const std::vector<float> &w);
std::vector<float> MeasureToWorld(const Measure &m);


#endif /*STATE_H*/