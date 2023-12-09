#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"

class State
{
public:
    typedef enum
    {
        not_selected,
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
        test_ir,
        test_odometory,
        test_rotation,
        test_slalom1,
        test_slalom2,
        test_slalom3,
        m_identification,
        step_identification,
        party_trick,
        error
    } Mode;

    typedef enum
    {
        slalom,
        m_iden,
        step_iden,
        pivot_turn,
        translation
    } Log;

    Function func;
    MazeLoad mazeload;
    Interruption interruption;
    Mode mode;
    Log log;

    State(Mode init_mode = select_function) : func(not_selected), mazeload(not_load), interruption(not_interrupt), mode(init_mode), log(slalom) {}
};

#endif /* _STATE_H_ */