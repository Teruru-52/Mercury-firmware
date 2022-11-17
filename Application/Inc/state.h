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
        FORWARD2,
        TURN_LEFT90,
        TURN_RIGHT90,
        PIVOT_TURN_RIGHT90,
        PIVOT_TURN_RIGHT90_2,
        PIVOT_TURN_LEFT90,
        PIVOT_TURN180,
        FRONT_WALL_CORRECTION,
        FRONT_WALL_CORRECTION2,
        BLIND_ALLEY,
        BACK,
        START_MOVE,
        SEARCH,
        CALC_PATH,
        OUTPUT,
        WAIT,
        ELSE
    } Mode;

    Interruption interruption;
    Mode mode;

    State(Mode init_mode = INITIALIZE) : interruption(NOT_INTERRUPT), mode(init_mode) {}
};

#endif /* _STATE_H_ */