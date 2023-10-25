#include "../../Inc/hardware/speaker.h"

namespace hardware
{
    void Speaker::Beep()
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
        HAL_Delay(30);
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

} // namespace hardware