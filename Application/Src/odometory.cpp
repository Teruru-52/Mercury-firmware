#include "odometory.h"

namespace undercarriage
{
  Odometory::Odometory(float sampling_period)
      : encoder(sampling_period),
        imu(sampling_period),
        sampling_period(sampling_period) {}

  void Odometory::Initialize()
  {
    imu.Initialize();
    imu.CalcOffset();
  }

  void Odometory::Reset()
  {
    cur_pos.clear();
    l = 0;
    encoder.Reset();
    ResetTheta();
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
    acc_x = imu.GetAccX();
    cur_pos.th = imu.GetAngle();
    cur_vel.th = imu.GetAngularVelocity();

    cur_vel.x = v;
    cur_vel.y = 0.0;

    // Euler method
    // cur_pos.x += v * cos(cur_pos.th) * sampling_period;
    // cur_pos.y += v * sin(cur_pos.th) * sampling_period;
    // l += v * sampling_period;

    // Bilinear transform
    cur_pos.x += (v + pre_v) * cos(cur_pos.th) * sampling_period * 0.5;
    cur_pos.y += (v + pre_v) * sin(cur_pos.th) * sampling_period * 0.5;
    l += (v + pre_v) * sampling_period * 0.5;
    pre_v = v;
  }

  void Odometory::UpdateIMU()
  {
    imu.Update();
  }

  ctrl::Pose Odometory::GetPosition()
  {
    return cur_pos;
  }

  ctrl::Pose Odometory::GetVelocity()
  {
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
    // printf("%f, %f\n", cur_pos.th, cur_vel.th);
    // printf("%f, %f\n", cur_pos.x, cur_pos.y);
  }
} //  namespace undercarriage