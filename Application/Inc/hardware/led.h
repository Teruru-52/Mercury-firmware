#ifndef HARDWARE_LED_HPP_
#define HARDWARE_LED_HPP_

#include "main.h"

namespace hardware
{
    class LED
    {
    public:
        void on_side_left();
        void on_front_left();
        void on_side_right();
        void on_front_right();

        void on_back_right();
        void on_back_left();

        void off_side_left();
        void off_front_left();
        void off_side_right();
        void off_front_right();

        void off_back_right();
        void off_back_left();

        void on_all();
        void off_all();

        void Func0();
        void Func1();
        void Func2();
        void Func3();
        void Func4();
        void Func5();
        void Func6();
        void Func7();
        void Func8();
        void Func9();
        void Func10();
        void Func11();
        void Func12();
        void Func13();
        void Func14();
        void Func15();

        void Flashing();
    };
} // namespace hardware
#endif //  HARDWARE_LED_HPP_