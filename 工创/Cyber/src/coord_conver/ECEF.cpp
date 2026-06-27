
#include "coord_conver/ECEF.h"

double ECEF::Epsilon = 1.0e-6;
double ECEF::a = 6378137.0; //WGS-84 semi-major axis
double ECEF::e2 = 6.6943799901377997e-3; //WGS-84 first eccentricity squared
double ECEF::a1 = 4.2697672707157535e+4; //a1 = a*e2
double ECEF::a2 = 1.8230912546075455e+9; //a2 = a1*a1
double ECEF::a3 = 1.4291722289812413e+2; //a3 = a1*e2/2
double ECEF::a4 = 4.5577281365188637e+9; //a4 = 2.5*a2
double ECEF::a5 = 4.2840589930055659e+4; //a5 = a1+a3
double ECEF::a6 = 9.9330562000986220e-1; //a6 = 1-e2
