/**
 * @file pid_controller.h
 * @author Teruru-52
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

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
    void Reset();
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

class PID_Instances
{
public:
    PID *angle;
    PID *rot_vel;
    PID *trans_vel;
    PID *ir_front_l;
    PID *ir_front_r;
    PID *ir_side;

    PID_Instances(PID *angle,
                  PID *rot_vel,
                  PID *trans_vel,
                  PID *ir_front_l,
                  PID *ir_front_r,
                  PID *ir_side)
        : angle(angle),
          rot_vel(rot_vel),
          trans_vel(trans_vel),
          ir_front_l(ir_front_l),
          ir_front_r(ir_front_r),
          ir_side(ir_side){};

    void Reset()
    {
        angle->Reset();
        rot_vel->Reset();
        trans_vel->Reset();
        ir_front_l->Reset();
        ir_front_r->Reset();
        ir_side->Reset();
    }
};

#endif //  PID_CONTROLLER_H