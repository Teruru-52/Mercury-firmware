#ifndef HARDWARE_ENCODER_H_
#define HARDWARE_ENCODER_H_

#include "main.h"

namespace hardware
{
    class Encoder
    {
    public:
        Encoder(float sampling_period);

        void Update();
        void Reset();
        void Update_L();
        void Update_R();
        int16_t GetPulseL();
        int16_t GetPulseR();
        float GetAngularVelocity(int16_t pulse);
        float GetAngle(int16_t pulse);
        float GetVelocity();
        float GetPosition();

    private:
        float gear_ratio = 11.0f / 43.0f;
        float sampling_period; // [s]
        // float tire_radius = 0.012; // [m]
        float tire_radius = 0.013; // [m]
        // float tire_radius = 0.0131; // [m]
        const float ppr = 2048.0;
        int16_t pulse_left;
        int16_t pulse_right;
    };
} // namespace hardware

#endif // HARDWARE_ENCODER_HPP_