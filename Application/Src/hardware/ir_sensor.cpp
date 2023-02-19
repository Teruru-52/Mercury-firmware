#include "hardware/ir_sensor.h"

namespace hardware
{
    IRsensor::IRsensor(float ir_start_base, float ir_wall_base)
        : ir_start_base(ir_start_base),
          ir_wall_base(ir_wall_base) {}

    void IRsensor::on_front_led()
    {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 50);
    }

    void IRsensor::on_side_led()
    {
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 50);
    }

    void IRsensor::off_front_led()
    {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    }

    void IRsensor::off_side_led()
    {
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
    }

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

    void IRsensor::UI_led_onoff()
    {
        if (ir_value.fl > ir_wall_base)
            led.on_front_left();
        else
            led.off_front_left();

        if (ir_value.fr > ir_wall_base)
            led.on_front_right();
        else
            led.off_front_right();

        if (ir_value.sl > ir_wall_base)
            led.on_side_left();
        else
            led.off_side_left();

        if (ir_value.sr > ir_wall_base)
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

    void IRsensor::StartDMA()
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_f, 3);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dma_b, 2);
    }

    void IRsensor::UpdateSideValue()
    {
        for (int i = sampling_count - 1; i > 0; i--)
        {
            sl[i] = sl[i - 1];
            sr[i] = sr[i - 1];
        }
        sl[0] = dma_b[0];
        sr[0] = dma_b[1];
    }

    void IRsensor::UpdateFrontValue()
    {
        for (int i = sampling_count - 1; i > 0; i--)
        {
            fl[i] = fl[i - 1];
            fr[i] = fr[i - 1];
        }
        fl[0] = dma_f[0];
        fr[0] = dma_f[1];
    }

    void IRsensor::Update()
    {
        uint32_t max_fl = 0;
        uint32_t max_fr = 0;
        uint32_t max_sl = 0;
        uint32_t max_sr = 0;

        for (int i = sampling_count - 1; i >= 0; i--)
        {
            if (fl[i] > max_fl)
                max_fl = fl[i];
            if (fr[i] > max_fr)
                max_fr = fr[i];
            if (sl[i] > max_sl)
                max_sl = sl[i];
            if (sr[i] > max_sr)
                max_sr = sr[i];
        }

        ir_value.fl = max_fl;
        ir_value.fr = max_fr;
        ir_value.sl = max_sl;
        ir_value.sr = max_sr;

        UI_led_onoff();
    }

    IR_Value IRsensor::GetIRSensorData()
    {
        return ir_value;
    }

    float IRsensor::GetBatteryVoltage()
    {
        bat_vol = (float)dma_f[2] * 3.3 / 4096.0 * 3.0;
        return bat_vol;
    }

    void IRsensor::BatteryCheck()
    {
        UpdateFrontValue();
        bat_vol = (float)dma_f[2] * 3.3 / 4096.0 * 3.0;
        if (bat_vol > 7.0)
        {
            led.on_side_right();
        }
        if (bat_vol > 6.0)
        {
            led.on_front_right();
        }
        if (bat_vol > 5.0)
        {
            led.on_front_left();
        }
        if (bat_vol > 4.0)
        {
            led.on_side_left();
        }
        printf("%f [V]\n", bat_vol);
    }

    bool IRsensor::StartInitialize()
    {
        if (ir_value.sl > ir_start_base && ir_value.sr > ir_start_base)
            return true;
        else
            return false;
    }
}
