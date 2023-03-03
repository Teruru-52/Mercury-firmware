#include "hardware/speaker.h"

namespace hardware
{
    void Speaker::Beep()
    {
        SpeakerOn();
        HAL_Delay(30);
        SpeakerOff();
    }

    void Speaker::SpeakerOn()
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
    }

    void Speaker::SpeakerOff()
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }

} // namespace hardware