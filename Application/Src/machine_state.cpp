/*
 * machine_state.cpp
 *
 *  Created on: January 13th, 2024
 *      Author: Reiji Terunuma
 */

#include "machine_state.h"

State::State()
    : func(not_selected),
      mazeload(not_load),
      interruption(not_interrupt),
      mode(select_function),
      log(slalom) {}

void State::SelectFunc(int16_t pulse)
{
    if (pulse < PULSE_DIFF)
    {
        led.Func0();
        func = func0;
    }
    else if (pulse < PULSE_DIFF * 2)
    {
        led.Func1();
        func = func1;
    }
    else if (pulse < PULSE_DIFF * 3)
    {
        led.Func2();
        func = func2;
    }
    else if (pulse < PULSE_DIFF * 4)
    {
        led.Func3();
        func = func3;
    }
    else if (pulse < PULSE_DIFF * 5)
    {
        led.Func4();
        func = func4;
    }
    else if (pulse < PULSE_DIFF * 6)
    {
        led.Func5();
        func = func5;
    }
    else if (pulse < PULSE_DIFF * 7)
    {
        led.Func6();
        func = func6;
    }
    else if (pulse < PULSE_DIFF * 8)
    {
        led.Func7();
        func = func7;
    }
    else if (pulse < PULSE_DIFF * 9)
    {
        led.Func8();
        func = func8;
    }
    else if (pulse < PULSE_DIFF * 10)
    {
        led.Func9();
        func = func9;
    }
    else if (pulse < PULSE_DIFF * 11)
    {
        led.Func10();
        func = func10;
    }
    else if (pulse < PULSE_DIFF * 12)
    {
        led.Func11();
        func = func11;
    }
    else if (pulse < PULSE_DIFF * 13)
    {
        led.Func12();
        func = func12;
    }
    else if (pulse < PULSE_DIFF * 14)
    {
        led.Func13();
        func = func13;
    }
    else if (pulse < PULSE_DIFF * 13)
    {
        led.Func12();
        func = func12;
    }
    else if (pulse < PULSE_DIFF * 14)
    {
        led.Func13();
        func = func13;
    }
    else if (pulse < PULSE_DIFF * 15)
    {
        led.Func14();
        func = func14;
    }
    else
    {
        led.Func15();
        func = func15;
    }
    if (func != pre_func)
        speaker.Beep();
    pre_func = func;
}

void State::SelectLoadMaze(int16_t pulse)
{
    if (pulse < PULSE_HALF)
    {
        led.off_back_right();
        mazeload = not_load;
    }
    else
    {
        led.on_back_right();
        mazeload = load;
    }
}