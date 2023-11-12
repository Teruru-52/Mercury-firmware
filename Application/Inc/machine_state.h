#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"

class State
{
public:
    typedef enum
    {
        func0,
        func1,
        func2,
        func3,
        func4,
        func5,
        func6,
        func7,
        func8,
        func9,
        func10,
        func11,
        func12,
        func13,
        func14,
        func15
    } Function;

    typedef enum
    {
        load,
        not_load
    } MazeLoad;

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
    MazeLoad mazeload;
    Interruption interruption;
    Mode mode;

    State(Mode init_mode = select_function) : func(func1), mazeload(not_load), interruption(not_interrupt), mode(init_mode) {}
};

#endif /* _STATE_H_ */