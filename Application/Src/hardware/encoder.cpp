#include "../../Inc/hardware/encoder.h"

#include <cmath>

namespace hardware
{
    Encoder::Encoder(float sampling_period)
        : sampling_period(sampling_period) {}

    void Encoder::Update()
    {
        Update_L();
        Update_R();
    }

    void Encoder::Reset()
    {
        TIM3->CNT = 0;
        TIM4->CNT = 0;
        position = 0.0;
    }

    void Encoder::Update_L()
    {
        pulse_left = 0;
        int16_t enc_buff = TIM3->CNT;
        TIM3->CNT = 0;

        if (enc_buff > 32767)
            pulse_left = (int16_t)enc_buff * -1;
        else
            pulse_left = (int16_t)enc_buff;
    }

    void Encoder::Update_R()
    {
        pulse_right = 0;
        int16_t enc_buff = TIM4->CNT;
        TIM4->CNT = 0;

        if (enc_buff > 32767)
            pulse_right = (int16_t)enc_buff;
        else
            pulse_right = (int16_t)enc_buff * -1;
    }

    int16_t Encoder::GetPulseL()
    {
        int16_t enc_buff = TIM3->CNT;
        pulse_left = (int16_t)enc_buff;
        if (pulse_left < 0)
            pulse_left += 32767;
        return pulse_left;
    }

    int16_t Encoder::GetPulseR()
    {
        int16_t enc_buff = TIM4->CNT;
        pulse_right = (int16_t)enc_buff * -1;
        if (pulse_right < 0)
            pulse_right += 32767;
        return pulse_right;
    }

    float Encoder::GetAngularVelocity(int16_t pulse)
    {
        return (float)pulse * (2.0 * M_PI / ppr) * gear_ratio / sampling_period / 4.0;
    }

    float Encoder::GetAngle(int16_t pulse)
    {
        return (float)pulse * (2.0 * M_PI / ppr) * gear_ratio / 4.0;
    }

    float Encoder::GetVelocity()
    {
        return (GetAngularVelocity(pulse_left) + GetAngularVelocity(pulse_right)) * tire_radius / 2.0;
    }

    float Encoder::GetPosition()
    {
        position += (GetAngle(pulse_left) + GetAngle(pulse_right)) * tire_radius / 2.0;
        return position;
    }
} // namespace hardware