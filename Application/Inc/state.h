#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"

class State
{
public:
    typedef enum
    {
        INTERRUPT,
        NOT_INTERRUPT
    } Interruption;

    typedef enum
    {
        INITIALIZE,
        FORWARD,
        TURN_LEFT90,
        TURN_RIGHT90,
        PIVOT_TURN_RIGHT90,
        PIVOT_TURN_LEFT90,
        PIVOT_TURN180,
        FRONT_WALL_CORRECTION,
        BLIND_ALLEY,
        BACK,
        OUTPUT,
        WAIT,
        ELSE
    } Mode;

    Interruption interruption;
    Mode mode;

    State(Mode init_mode = INITIALIZE) : interruption(NOT_INTERRUPT), mode(init_mode) {}
};

#endif /* _STATE_H_ */