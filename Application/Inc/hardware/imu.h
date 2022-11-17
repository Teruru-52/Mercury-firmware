#ifndef HARDWARE_IMU_HPP_
#define HARDWARE_IMU_HPP_

#include "main.h"

namespace hardware
{
    class IMU
    {
    public:
        IMU(float sampling_period);

        uint8_t read_byte(uint8_t reg);
        void write_byte(uint8_t reg, uint8_t data);

        void Initialize();
        void CalcOffset();
        void Update();
        void UpdateGyro();
        void UpdateAcc();
        float GetAngle();
        float GetAngularVelocity();
        float GetAccX();
        void ResetTheta();

    private:
        float sampling_period;
        // default
        // const float gyro_factor = 16.4;
        const float gyro_factor = 16.6;
        const float acc_factor = 8192.0;

        float theta;
        float gyro_z;
        float pre_gyro_z;
        float acc_x;
        float offset_gz;
    };
}
#endif // HARDWARE_IMU_HPP_