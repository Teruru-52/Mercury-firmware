/**
 * @file speaker.h
 * @author Teruru-52
 */

#ifndef HARDWARE_SPEAKER_H_
#define HARDWARE_SPEAKER_H_

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

extern hardware::Speaker speaker;

#endif //  HARDWARE_SPEAKER_H_