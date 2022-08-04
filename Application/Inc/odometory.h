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
        float x;
        float y;
        float theta;
        std::vector<float> cur_pos{0, 0, 0};
        std::vector<float> cur_vel{0, 0};

    public:
        Odometory(float sampling_period);

        void Initialize();
        void Update();
        void IMU_Update();
        void Reset();
        std::vector<float> GetPosition();
        std::vector<float> GetVelocity();
        void OutputLog();
    };
} //  namespace undercarriage
#endif //  ODOMETORY_HPP_