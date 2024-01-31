#ifndef HARDWARE_IR_SENSOR_HPP_
#define HARDWARE_IR_SENSOR_HPP_

#include "main.h"
#include "led.h"

namespace hardware
{
    struct IR_Value
    {
        uint32_t fl;
        uint32_t fr;
        uint32_t sl;
        uint32_t sr;
    };

    struct IR_Base
    {
        float fl;
        float fr;
        float sl;
        float sr;
        float slalom;
    };

    struct IR_FrontParam
    {
        float a;
        float b;
        float c;
        float d;
    };

    class IRsensor
    {
    private:
        LED led;
        const int sampling_count = 84;
        float ir_start_base;

        uint16_t dma_f[3];
        uint16_t dma_b[2];

        uint32_t max_fl = 0;
        uint32_t max_fr = 0;
        uint32_t max_sl = 0;
        uint32_t max_sr = 0;

        IR_Value ir_value;
        IR_Base *ir_is_wall;
        float bat_vol;

    public:
        IRsensor(float ir_start_base, IR_Base *ir_is_wall);

        void on_front_led() { __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 50); };
        void on_side_led() { __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 50); };
        void off_front_led() { __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0); };
        void off_side_led() { __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0); };
        void on_all_led();
        void off_all_led();

        void UI_led_onoff(const IR_Value &ir_value);
        void UI_led_off();
        void PrintWalldata(const IR_Value &ir_value);

        void StartDMA();
        void UpdateSideValue();
        void UpdateFrontValue();
        void Update();
        IR_Value GetIRSensorData() { return ir_value; };
        float GetBatteryVoltage();
        void BatteryCheck();
        bool StartInitialize();
        void OutputLog();
    };
}
#endif //  HARDWARE_IR_SENSOR_HPP_