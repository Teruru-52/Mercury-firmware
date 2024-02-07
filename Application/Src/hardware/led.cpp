/**
 * @file led.cpp
 * @author Teruru-52
 */

#include "hardware/led.h"

hardware::LED led;

namespace hardware
{
    void LED::on_side_left()
    {
        Write_GPIO(SIDE_LEFT_LED, GPIO_PIN_SET);
    }

    void LED::on_front_left()
    {
        Write_GPIO(FRONT_LEFT_LED, GPIO_PIN_SET);
    }

    void LED::on_side_right()
    {
        Write_GPIO(SIDE_RIGHT_LED, GPIO_PIN_SET);
    }

    void LED::on_front_right()
    {
        Write_GPIO(FRONT_RIGHT_LED, GPIO_PIN_SET);
    }

    void LED::on_back_right()
    {
        Write_GPIO(BACK_RIGHT_LED, GPIO_PIN_SET);
    }

    void LED::on_back_left()
    {
        Write_GPIO(BACK_LEFT_LED, GPIO_PIN_SET);
    }

    void LED::off_side_left()
    {
        Write_GPIO(SIDE_LEFT_LED, GPIO_PIN_RESET);
    }

    void LED::off_front_left()
    {
        Write_GPIO(FRONT_LEFT_LED, GPIO_PIN_RESET);
    }

    void LED::off_side_right()
    {
        Write_GPIO(SIDE_RIGHT_LED, GPIO_PIN_RESET);
    }

    void LED::off_front_right()
    {
        Write_GPIO(FRONT_RIGHT_LED, GPIO_PIN_RESET);
    }

    void LED::off_back_right()
    {
        Write_GPIO(BACK_RIGHT_LED, GPIO_PIN_RESET);
    }

    void LED::off_back_left()
    {
        Write_GPIO(BACK_LEFT_LED, GPIO_PIN_RESET);
    }

    void LED::on_all()
    {
        on_side_left();
        on_front_left();
        on_side_right();
        on_front_right();
    }

    void LED::off_all()
    {
        off_side_left();
        off_front_left();
        off_side_right();
        off_front_right();
    }

    void LED::Func0()
    {
        off_all();
    }

    void LED::Func1()
    {
        on_side_right();
        off_front_right();
        off_front_left();
        off_side_left();
    }

    void LED::Func2()
    {
        off_side_right();
        on_front_right();
        off_front_left();
        off_side_left();
    }

    void LED::Func3()
    {
        on_side_right();
        on_front_right();
        off_front_left();
        off_side_left();
    }

    void LED::Func4()
    {
        off_side_right();
        off_front_right();
        on_front_left();
        off_side_left();
    }

    void LED::Func5()
    {
        on_side_right();
        off_front_right();
        on_front_left();
        off_side_left();
    }

    void LED::Func6()
    {
        off_side_right();
        on_front_right();
        on_front_left();
        off_side_left();
    }

    void LED::Func7()
    {
        on_side_right();
        on_front_right();
        on_front_left();
        off_side_left();
    }

    void LED::Func8()
    {
        off_side_right();
        off_front_right();
        off_front_left();
        on_side_left();
    }

    void LED::Func9()
    {
        on_side_right();
        off_front_right();
        off_front_left();
        on_side_left();
    }

    void LED::Func10()
    {
        off_side_right();
        on_front_right();
        off_front_left();
        on_side_left();
    }

    void LED::Func11()
    {
        on_side_right();
        on_front_right();
        off_front_left();
        on_side_left();
    }

    void LED::Func12()
    {
        off_side_right();
        off_front_right();
        on_front_left();
        on_side_left();
    }

    void LED::Func13()
    {
        on_side_right();
        off_front_right();
        on_front_left();
        on_side_left();
    }

    void LED::Func14()
    {
        off_side_right();
        on_front_right();
        on_front_left();
        on_side_left();
    }

    void LED::Func15()
    {
        on_all();
    }

    void LED::Flashing()
    {
        for (int i = 0; i < 2; i++)
        {
            on_all();
            HAL_Delay(500);
            off_all();
            HAL_Delay(500);
        }
    }

}
