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
    ResetEncoder();
    Reset();
  }

  void Odometory::Reset()
  {
    cur_pos.x = 0;
    cur_pos.y = 0;
    length = 0;
    pre_vel_x = 0;
    // ResetTheta();
  }

  void Odometory::ResetTheta()
  {
    cur_pos.th = 0.0;
    imu.ResetTheta();
  }

  void Odometory::Update()
  {
    imu.Update();
    encoder.Update();

    vel_x = encoder.GetVelocity();
    acc_x = imu.GetAccX();
    cur_pos.th = imu.GetAngle();
    cur_vel.th = imu.GetAngularVelocity();

    cur_vel.x = vel_x;
    cur_vel.y = 0.0;

    // Bilinear transform
    // cur_pos.x += (vel_x + pre_vel_x) * cos(cur_pos.th) * sampling_period * 0.5;
    // cur_pos.y += (vel_x + pre_vel_x) * sin(cur_pos.th) * sampling_period * 0.5;
    // length += (vel_x + pre_vel_x) * sampling_period * 0.5;
    // pre_vel_x = vel_x;

    length += encoder.GetPosition();
    cur_pos.x = length * cos(cur_pos.th);
    cur_pos.y = length * sin(cur_pos.th);
  }

  void Odometory::OutputLog()
  {
    printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", cur_pos.x, cur_pos.y, cur_pos.th, length, acc_x);
    // printf("%f, %f\n", cur.pos[2], cur.vel[1]);
    // printf("%f\n", acc_x);
    // printf("%f, %f\n", cur_pos.x, cur_pos.y);
    // printf("%f, %f\n", cur_pos.th, cur_vel.th);
  }
} //  namespace undercarriage