#include "../Inc/odometory.h"

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
        l(0) {}

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
    ResetTheta();
  }

  void Odometory::ResetTheta()
  {
    imu.ResetTheta();
  }

  void Odometory::Update()
  {
    encoder.Update();

    v = encoder.GetVelocity();
    omega = imu.GetAngularVelocity();
    theta = imu.GetAngle();
    x_local += v * cos(theta) * sampling_period;
    y_local += v * sin(theta) * sampling_period;
    // x_gloabl += x_local;
    // y_global += y_local;
    l += v * sampling_period;
  }

  void Odometory::UpdateIMU()
  {
    imu.Update();
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