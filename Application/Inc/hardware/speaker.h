#ifndef HARDWARE_SPEAKER_HPP_
#define HARDWARE_SPEAKER_HPP_

#include "main.h"

namespace hardware
{
    class Speaker
    {
    private:
        bool flag_speaker = false;

    public:
        void Beep();
        void SpeakerOn();
        void SpeakerOff();
        void ToggleSpeaker();
    };
} // namespace hardware

#endif //  HARDWARE_SPEAKER_HPP_