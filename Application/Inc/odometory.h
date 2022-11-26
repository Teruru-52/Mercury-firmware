#ifndef ODOMETORY_HPP_
#define ODOMETORY_HPP_

#include "main.h"
#include <vector>
#include <cmath>
#include "hardware/encoder.h"
#include "hardware/imu.h"

namespace undercarriage
{
    class Odometory
    {
    private:
        hardware::Encoder encoder;
        hardware::IMU imu;

        float sampling_period; // [s]
        float v;
        float omega;
        float x_global;
        float y_global;
        float x_local;
        float y_local;
        float theta;
        float l;
        std::vector<float> cur_pos{0, 0, 0};
        std::vector<float> cur_vel{0, 0};

    public:
        Odometory(float sampling_period);

        void Initialize();
        void Update();
        void UpdateIMU();
        void Reset();
        void ResetTheta();
        int16_t GetPulse();
        std::vector<float> GetPosition();
        std::vector<float> GetVelocity();
        float GetLength();
        void OutputLog();
    };
} //  namespace undercarriage
#endif //  ODOMETORY_HPP_