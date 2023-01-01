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
        SEARCH,
        CALC_PATH,
        RUN_SEQUENCE,
        SPEAKER,
        LED,
        OUTPUT,
        ELSE
    } Mode;

    Function func;
    Interruption interruption;
    Mode mode;

    State(Mode init_mode = SELECT_FUNCTION) : func(FUNC0), interruption(NOT_INTERRUPT), mode(init_mode) {}
};

#endif /* _STATE_H_ */