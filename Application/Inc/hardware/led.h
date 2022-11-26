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

        void Flashing();
    };
} // namespace hardware
#endif //  HARDWARE_LED_HPP_