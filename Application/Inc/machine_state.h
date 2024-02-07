/**
 * @file machine_state.h
 * @author Teruru-52
 */

#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"
#include "hardware/led.h"
#include "hardware/speaker.h"

#define PULSE_DIFF 2048
#define PULSE_HALF 16384

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
        test_translation2,
        test_translation1,
        test_slalom1,
        test_slalom2,
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
        translation,
        maze,
    } Log;

    Function func;
    MazeLoad mazeload;
    Interruption interruption;
    Mode mode;
    Log log;

    State();
    void SelectFunc(int16_t pulse);
    void SelectLoadMaze(int16_t pulse);

private:
    Function pre_func = not_selected;
};

#endif /* _STATE_H_ */