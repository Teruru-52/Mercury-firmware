/**
 * @file encoder.cpp
 * @author Reiji Terunuma
 */

#include "hardware/encoder.h"

#include <cmath>

namespace hardware
{
    Encoder::Encoder(float sampling_period)
        : sampling_period(sampling_period)
    {
        coeff_pulse2angle = 2.0f * M_PI * gear_ratio * 0.25f;
        coeff_pulse2vel = coeff_pulse2angle / sampling_period;
    }

    void Encoder::Update()
    {
        Update_L();
        Update_R();
    }

    void Encoder::Reset()
    {
        TIM3->CNT = 0;
        TIM4->CNT = 0;
    }

    void Encoder::Update_L()
    {
        // __disable_irq();
        pulse_left = 0;
        int16_t enc_buff = TIM4->CNT;
        TIM4->CNT = 0;

        if (enc_buff > 32767)
            pulse_left = (int16_t)enc_buff * -1;
        else
            pulse_left = (int16_t)enc_buff;
        // __enable_irq();
    }

    void Encoder::Update_R()
    {
        // __disable_irq();
        pulse_right = 0;
        int16_t enc_buff = TIM3->CNT;
        TIM3->CNT = 0;

        if (enc_buff > 32767)
            pulse_right = (int16_t)enc_buff;
        else
            pulse_right = (int16_t)enc_buff * -1;
        // __enable_irq();
    }

    int16_t Encoder::GetPulseL()
    {
        __disable_irq();
        int16_t enc_buff = TIM4->CNT;
        pulse_left = (int16_t)enc_buff;
        if (pulse_left < 0)
            pulse_left += 32767;
        __enable_irq();
        return pulse_left;
    }

    int16_t Encoder::GetPulseR()
    {
        __disable_irq();
        int16_t enc_buff = TIM3->CNT;
        pulse_right = (int16_t)enc_buff * -1;
        if (pulse_right < 0)
            pulse_right += 32767;
        __enable_irq();
        return pulse_right;
    }

    float Encoder::GetAngularVelocity(int16_t pulse)
    {
        return static_cast<float>(pulse) / ppr * coeff_pulse2vel;
    }

    float Encoder::GetAngle(int16_t pulse)
    {
        return static_cast<float>(pulse) / ppr * coeff_pulse2angle;
    }

    float Encoder::GetVelocity()
    {
        return (GetAngularVelocity(pulse_left) + GetAngularVelocity(pulse_right)) * tire_radius * 0.5f;
    }

    float Encoder::GetPosition()
    {
        return (GetAngle(pulse_left) + GetAngle(pulse_right)) * tire_radius * 0.5f;
    }
} // namespace hardware