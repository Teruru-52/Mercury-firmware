#ifndef ODOMETORY_H_
#define ODOMETORY_H_

#include "main.h"
#include <vector>
#include <cmath>
#include "hardware/encoder.h"
#include "hardware/imu.h"
#include "state.h"

namespace undercarriage
{
    class Odometory
    {
    private:
        hardware::Encoder encoder;
        hardware::IMU imu;

        float sampling_period; // [s]
        ctrl::State robot_state;
        ctrl::State local_state;
        // ctrl::State global_state;
        float v;
        float omega;
        float acc_x;
        float x_global;
        float y_global;
        float x_local;
        float y_local;
        float theta;
        float l;
        float pre_v;
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
        ctrl::State GetState();
        std::vector<float> GetPosition();
        std::vector<float> GetVelocity();
        float GetAccX();
        float GetLength();
        void OutputLog();
    };
} //  namespace undercarriage
#endif //  ODOMETORY_H_