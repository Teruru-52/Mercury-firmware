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
        BACK,
        OUTPUT,
        WAIT,
        ELSE
    } Mode;

    Interruption interruption;
    Mode mode;

    State() : interruption(NOT_INTERRUPT), mode(INITIALIZE) {}
};

#endif /* _STATE_H_ */