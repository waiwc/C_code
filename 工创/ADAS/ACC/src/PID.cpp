#include "PID.hpp"

PID::PID() :kp(0), ki(0), kd(0), e_pre_1(0), e_pre_2(0), target(0), actual(0)
{
	A = kp + ki + kd;
	B = -2 * kd - kp;
	C = kd;
	e = target - actual;
}
PID::PID(double p, double i, double d) :kp(p), ki(i), kd(d), e_pre_1(0), e_pre_2(0), target(0), actual(0)
{
	A = kp + ki + kd;
	B = -2 * kd - kp;
	C = kd;
	e = target - actual;
}
PID::~PID()
{
}
double PID::pidControl(double tar, double act)
{
	double u_increment;
	target = tar;
	actual = act;
	e = target - actual;
	u_increment = A * e + B * e_pre_1 + C * e_pre_2;
	// add saturation function
	e_pre_2 = e_pre_1;
	e_pre_1 = e;
	return u_increment;
}
