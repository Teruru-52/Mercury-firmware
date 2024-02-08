/**
 * @file speaker.h
 * @brief Speaker for debugging and UI, and for indicating the state of the robot
 * @author Teruru-52
 */

#ifndef HARDWARE_SPEAKER_H_
#define HARDWARE_SPEAKER_H_

#include "main.h"

/**
 * @brief namespace for hardware
 */
namespace hardware
{
    /**
     * @brief class for UI speaker
     */
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