#include "../../Inc/hardware/motor.h"

namespace hardware
{
    void Motor::UpdateBatteryVoltage(float bat_vol)
    {
        this->bat_vol = bat_vol;
    }

    int Motor::GetDuty(float input_vol)
    {
        // return max_input * input_vol / battery.GetBatteryVoltage();
        return (int)((float)max_input * input_vol / bat_vol);
    }

    void Motor::Drive(float v_left, float v_right)
    {
        duty_left = GetDuty(v_left);
        duty_right = GetDuty(v_right);

        if (duty_left > max_input)
            duty_left = max_input;
        else if (duty_left < -max_input)
            duty_left = max_input;

        if (duty_right > max_input)
            duty_right = max_input;
        else if (duty_right < -max_input)
            duty_right = -max_input;

        if (duty_left > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, max_input - duty_left);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, max_input);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, max_input);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, max_input + duty_left);
        }

        if (duty_right > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, max_input);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, max_input - duty_right);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, max_input + duty_right);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, max_input);
        }
    }

    void Motor::Brake()
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, max_input);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, max_input);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, max_input);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, max_input);
    }

    void Motor::Free()
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    }

}