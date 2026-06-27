#pragma once
#ifndef PID_H
#define PID_H
class PID
{
private:
	double kp;
	double ki;
	double kd;
	double target;
	double actual;
	double e;
	double e_pre_1;
	double e_pre_2;
	double A;
	double B;
	double C;
public:
	PID();
	PID(double p, double i, double d);
	~PID();
	double pidControl(double tar, double act);
};
#endif /* PID_H */
