#include "odometory.h"

namespace undercarriage
{
  Odometory::Odometory(float sampling_period)
      : encoder(sampling_period),
        imu(sampling_period),
        sampling_period(sampling_period),
        x_global(0),
        y_global(0),
        x_local(0),
        y_local(0),
        l(0),
        pre_v(0) {}

  void Odometory::Initialize()
  {
    imu.Initialize();
    imu.CalcOffset();
  }

  void Odometory::Reset()
  {
    x_local = 0;
    y_local = 0;
    l = 0;
    encoder.Reset();
    // ResetTheta();
  }

  void Odometory::ResetTheta()
  {
    imu.ResetTheta();
  }

  int16_t Odometory::GetPulse()
  {
    int16_t pulse = encoder.GetPulse();
    return pulse;
  }

  void Odometory::Update()
  {
    encoder.Update();

    v = encoder.GetVelocity();
    omega = imu.GetAngularVelocity();
    acc_x = imu.GetAccX();
    theta = imu.GetAngle();

    // robot_state.dq.x = v;
    // robot_state.dq.th = omega;
    // robot_state.ddq.x = acc_x;
    // local_state.dq.x = v * cos(theta);
    // local_state.dq.y = v * sin(theta);
    // local_state.dq.th = omega;
    // local_state.q.x += (v + pre_v) * cos(theta) * sampling_period * 0.5;
    // local_state.q.y += (v + pre_v) * sin(theta) * sampling_period * 0.5;
    // local_state.q.th = theta;

    // Euler method
    // x_local += v * cos(theta) * sampling_period;
    // y_local += v * sin(theta) * sampling_period;
    // x_gloabl += x_local;
    // y_global += y_local;
    // l += v * sampling_period;

    // Bilinear transform
    x_local += (v + pre_v) * cos(theta) * sampling_period * 0.5;
    y_local += (v + pre_v) * sin(theta) * sampling_period * 0.5;
    l += (v + pre_v) * sampling_period * 0.5;
    pre_v = v;
  }

  void Odometory::UpdateIMU()
  {
    imu.Update();
  }

  ctrl::State Odometory::GetState()
  {
    return local_state;
  }

  std::vector<float> Odometory::GetPosition()
  {
    cur_pos[0] = x_local;
    cur_pos[1] = y_local;
    cur_pos[2] = theta;
    return cur_pos;
  }

  std::vector<float> Odometory::GetVelocity()
  {
    cur_vel[0] = v;
    cur_vel[1] = omega;
    return cur_vel;
  }

  float Odometory::GetAccX()
  {
    return acc_x;
  }

  float Odometory::GetLength()
  {
    return l;
  }

  void Odometory::OutputLog()
  {
    // printf("%f, %f\n", theta, omega);
    // printf("%f, %f\n", x, y);
  }
} //  namespace undercarriage