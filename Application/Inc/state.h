#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"

class State
{
public:
    typedef enum
    {
        FUNC1,
        FUNC2,
        FUNC3,
        FUNC4,
        FUNC5,
    } Function;

    typedef enum
    {
        INTERRUPT,
        NOT_INTERRUPT
    } Interruption;

    typedef enum
    {
        SELECT_FUNCTION,
        SEARCH,
        RUN_SEQUENCE,
        OUTPUT,
        TEST
    } Mode;

    Function func;
    Interruption interruption;
    Mode mode;

    State(Mode init_mode = SELECT_FUNCTION) : func(FUNC1), interruption(NOT_INTERRUPT), mode(init_mode) {}
};

#endif /* _STATE_H_ */