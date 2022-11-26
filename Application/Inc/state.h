#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"

class State
{
public:
    typedef enum
    {
        FUNC0,
        FUNC1,
        FUNC2,
        FUNC3
    } Function;

    typedef enum
    {
        INTERRUPT,
        NOT_INTERRUPT
    } Interruption;

    typedef enum
    {
        SELECT_FUNCTION,
        INITIALIZE,
        INITIALIZE_POSITION,
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
        SPEAKER,
        LED,
        OUTPUT,
        WAIT,
        ELSE
    } Mode;

    Function func;
    Interruption interruption;
    Mode mode;

    State(Mode init_mode = SELECT_FUNCTION) : func(FUNC0), interruption(NOT_INTERRUPT), mode(init_mode) {}
};

#endif /* _STATE_H_ */