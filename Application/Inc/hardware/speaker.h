#ifndef HARDWARE_SPEAKER_HPP_
#define HARDWARE_SPEAKER_HPP_

#include "main.h"

namespace hardware
{
    class Speaker
    {
    public:
        void Beep();
        void SpeakerOn();
        void SpeakerOff();
    };
} // namespace hardware

#endif //  HARDWARE_SPEAKER_HPP_