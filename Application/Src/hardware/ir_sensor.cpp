/**
 * @file ir_sensor.cpp
 * @author Reiji Terunuma
 */

#include "hardware/ir_sensor.h"

namespace hardware
{
    IRsensor::IRsensor(float ir_start_base, IR_Base *ir_is_wall)
        : ir_start_base(ir_start_base),
          ir_is_wall(ir_is_wall) {}

    void IRsensor::on_all_led()
    {
        on_front_led();
        on_side_led();
    }

    void IRsensor::off_all_led()
    {
        off_front_led();
        off_side_led();
    }

    void IRsensor::UI_led_onoff(const IR_Value &ir_value)
    {
        if (ir_value.fl > ir_is_wall->fl)
            led.on_front_left();
        else
            led.off_front_left();

        if (ir_value.fr > ir_is_wall->fr)
            led.on_front_right();
        else
            led.off_front_right();

        if (ir_value.sl > ir_is_wall->sl)
            led.on_side_left();
        else
            led.off_side_left();

        if (ir_value.sr > ir_is_wall->sr)
            led.on_side_right();
        else
            led.off_side_right();
    }

    void IRsensor::UI_led_off()
    {
        led.off_front_left();
        led.off_front_right();
        led.off_side_left();
        led.off_side_right();
    }

    void IRsensor::PrintWalldata(const IR_Value &ir_value)
    {
        if (ir_value.sl > ir_is_wall->sl)
            printf("1");
        else
            printf("0");
        if (ir_value.fl > ir_is_wall->fl)
            printf("1");
        else
            printf("0");
        if (ir_value.fr > ir_is_wall->fr)
            printf("1");
        else
            printf("0");
        if (ir_value.sr > ir_is_wall->sr)
            printf("1\n");
        else
            printf("0\n");
    }

    void IRsensor::StartDMA()
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_f, 3);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dma_b, 2);
    }

    void IRsensor::UpdateSideValue()
    {
        if (dma_b[0] > max_sl)
            max_sl = dma_b[0];
        if (dma_b[1] > max_sr)
            max_sr = dma_b[1];
    }

    void IRsensor::UpdateFrontValue()
    {
        if (dma_f[0] > max_fl)
            max_fl = dma_f[0];
        if (dma_f[1] > max_fr)
            max_fr = dma_f[1];
    }

    void IRsensor::Update()
    {
        ir_value.fl = max_fl;
        ir_value.fr = max_fr;
        ir_value.sl = max_sl;
        ir_value.sr = max_sr;

        max_fl = 0;
        max_fr = 0;
        max_sl = 0;
        max_sr = 0;
    }

    float IRsensor::GetBatteryVoltage()
    {
        bat_vol = static_cast<float>(dma_f[2]) * 3.3 / 4095.0 * 3.0;
        return bat_vol;
    }

    void IRsensor::BatteryCheck()
    {
        UpdateFrontValue();
        bat_vol = static_cast<float>(dma_f[2]) * 3.3 / 4095.0 * 3.0;
        if (bat_vol > 8.0)
        {
            led.on_side_right();
        }
        if (bat_vol > 7.75)
        {
            led.on_front_right();
        }
        if (bat_vol > 7.5)
        {
            led.on_front_left();
        }
        if (bat_vol > 7.25)
        {
            led.on_side_left();
        }
        printf("battery: %.2f [V]\n", bat_vol);
    }

    bool IRsensor::StartInitialize()
    {
        if (dma_b[0] > ir_start_base && dma_b[1] > ir_start_base)
            return true;
        else
            return false;
    }

    void IRsensor::OutputLog()
    {
        printf("%lu, %lu, %lu, %lu\n", ir_value.fl, ir_value.fr, ir_value.sl, ir_value.sr);
    }
}
