/**
 * @file speaker.cpp
 * @brief Speaker for debugging and UI, and for indicating the state of the robot
 * @author Teruru-52
 */

#include "hardware/speaker.h"

hardware::Speaker speaker;

namespace hardware
{
    void Speaker::Beep()
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
        HAL_Delay(20);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }

    void Speaker::SpeakerOn()
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    }

    void Speaker::SpeakerOff()
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }

    void Speaker::ToggleSpeaker()
    {
        if (flag_speaker)
        {
            flag_speaker = false;
            SpeakerOff();
        }
        else
        {
            flag_speaker = true;
            SpeakerOn();
        }
    }

} // namespace hardware