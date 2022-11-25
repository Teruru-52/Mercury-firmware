#ifndef HARDWARE_MOTOR_HPP_
#define HARDWARE_MOTOR_HPP_

#include "main.h"
#include "ir_sensor.h"

namespace hardware
{
    class Motor
    {
    private:
        // IRsensor battery;

        float bat_vol;
        int max_input = 1600 - 1;
        int duty_left;
        int duty_right;

    public:
        // Motor();

        void UpdateBatteryVoltage(float bat_vol);
        int GetDuty(float input_vol);
        void Drive(float v_left, float v_right);
        void Brake();
        void Free();
    };
} // namespace hardware
#endif //  HARDWARE_MOTOR_HPP_