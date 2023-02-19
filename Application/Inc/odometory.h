#ifndef ODOMETORY_H_
#define ODOMETORY_H_

#include "main.h"
#include <cmath>
#include "hardware/encoder.h"
#include "hardware/imu.h"
#include "pose.h"

namespace undercarriage
{
    class Odometory
    {
    private:
        hardware::Encoder encoder;
        hardware::IMU imu;

        float sampling_period;       // [s]
        ctrl::Pose cur_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose cur_vel{0, 0, 0}; // robot coordinates
        float v;
        float pre_v{0.0};
        float acc_x;
        float l{0.0};

    public:
        Odometory(float sampling_period);

        void Initialize();
        void Update();
        void UpdateIMU();
        void Reset();
        void ResetTheta();
        int16_t GetPulse();
        ctrl::Pose GetPosition();
        ctrl::Pose GetVelocity();
        float GetAccX();
        float GetLength();
        void OutputLog();
    };
} //  namespace undercarriage
#endif //  ODOMETORY_H_