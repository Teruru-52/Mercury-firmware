#include "../../Inc/hardware/ir_sensor.h"

namespace hardware
{
    IRsensor::IRsensor(uint32_t threshold)
        : threshold(threshold),
          flag(false) {}

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
        if (ir_fl > threshold)
            led.on_front_left();
        else
            led.off_front_left();

        if (ir_fr > threshold)
            led.on_front_right();
        else
            led.off_front_right();

        if (ir_sl > threshold)
            led.on_side_left();
        else
            led.off_side_left();

        if (ir_sr > threshold)
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

    void IRsensor::UpdateSideValue()
    {
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dma_b, 2);

        for (int i = sampling_count - 1; i > 0; i--)
        {
            sl[i] = sl[i - 1];
            sr[i] = sr[i - 1];
        }
        sl[0] = dma_b[0];
        sr[0] = dma_b[1];

        ir_sl = dma_b[0];
        ir_sr = dma_b[1];
    }

    void IRsensor::UpdateFrontValue()
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_f, 3);

        for (int i = sampling_count - 1; i > 0; i--)
        {
            fl[i] = fl[i - 1];
            fr[i] = fr[i - 1];
        }
        fl[0] = dma_f[0];
        fr[0] = dma_f[1];

        ir_fl = dma_f[0];
        ir_fr = dma_f[1];

        bat_vol = (float)dma_f[2] * 3.3 / 4096.0 * 3.0;
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

        ir_fl = max_fl;
        ir_fr = max_fr;
        ir_sl = max_sl;
        ir_sr = max_sr;

        UI_led_onoff();
    }

    std::vector<uint32_t> IRsensor::GetIRSensorData()
    {
        ir_data[0] = ir_fl;
        ir_data[1] = ir_fr;
        ir_data[2] = ir_sl;
        ir_data[3] = ir_sr;
        return ir_data;
    }

    float IRsensor::GetBatteryVoltage()
    {
        return bat_vol;
    }

    void IRsensor::BatteryCheck()
    {
        UpdateFrontValue();
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
    }

    bool IRsensor::StartInitialize()
    {
        if (ir_fl > 2700 && ir_fr > 2700)
            return true;
        else
            return false;
    }

    bool IRsensor::GetFrontWallFlag()
    {
        if (ir_fl > 2100 && ir_fr > 2100)
        {
            flag = true;
        }
        return flag;
    }
}
