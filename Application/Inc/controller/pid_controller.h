#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "main.h"

// 後退差分による離散化

class Integrator
{
public:
    Integrator(float control_period);

    float Update(float error);
    void ResetIntegrator();

private:
    float control_period;
    float error_sum;
    float pre_error;
};

class Differentiator
{
public:
    Differentiator(float tf, float control_period);

    float Update(float error);
    void ResetDifferentiator();

private:
    float tf;
    float control_period;
    float coeff;
    float deriv;
    float pre_error;
    float pre_deriv;
};

class PID
{
public:
    PID(float kp, float ki, float kd, float tf, float control_period);

    float Update(float error);
    void ResetPID();
    void OutputLog();

private:
    float kp;
    float ki;
    float kd;
    float error;
    float sum;
    float deriv;
    float input;
    Integrator integrator;
    Differentiator differentiator;
};
#endif //  PID_CONTROLLER_HPP