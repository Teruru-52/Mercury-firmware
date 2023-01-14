#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"

class State
{
public:
    typedef enum
    {
        func1,
        func2,
        func3,
        func4,
        func5,
    } Function;

    typedef enum
    {
        interrupt,
        not_interrupt
    } Interruption;

    typedef enum
    {
        select_function,
        search,
        run_sequence,
        output,
        test,
        error
    } Mode;

    Function func;
    Interruption interruption;
    Mode mode;

    State(Mode init_mode = select_function) : func(func1), interruption(not_interrupt), mode(init_mode) {}
};

#endif /* _STATE_H_ */