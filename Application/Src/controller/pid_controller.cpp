#include "../../Inc/controller/pid_controller.h"

Integrator::Integrator(float control_period)
    : control_period(control_period),
      error_sum(0),
      pre_error(0) {}

float Integrator::Update(float error)
{
    // Euler method
    // error_sum += error * control_period;

    // Bilinear transform
    error_sum += (error + pre_error) * control_period * 0.5;
    pre_error = error;

    return error_sum;
}

void Integrator::ResetIntegrator()
{
    error_sum = 0.0;
    pre_error = 0.0;
}

Differentiator::Differentiator(float tf, float control_period)
    : tf(tf),
      control_period(control_period),
      coeff(tf / (tf + control_period)),
      deriv(0),
      pre_error(0),
      pre_deriv(0) {}

float Differentiator::Update(float error)
{
    // Euler method
    // float deriv = (error - pre_error) / control_period;

    // Bilinear transform
    deriv = (error - pre_error) * 2.0f / control_period - pre_deriv;

    // Low pass Filter (first order)
    pre_deriv = coeff * pre_deriv + (1.0f - coeff) * deriv;
    pre_error = error;
    return pre_deriv;
}

void Differentiator::ResetDifferentiator()
{
    pre_error = 0.0;
    pre_deriv = 0.0;
}

PID::PID(float kp, float ki, float kd, float tf, float control_period)
    : kp(kp),
      ki(ki),
      kd(kd),
      integrator(control_period),
      differentiator(tf, control_period) {}

float PID::Update(float error)
{
    this->error = error;
    sum = integrator.Update(error);
    deriv = differentiator.Update(error);
    input = kp * error + ki * sum + kd * deriv;
    return input;
}

void PID::ResetPID()
{
    integrator.ResetIntegrator();
    differentiator.ResetDifferentiator();
}

void PID::OutputLog()
{
    printf("%f, %f\n", error, input);
}