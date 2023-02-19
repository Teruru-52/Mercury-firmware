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
        float is_wall;
        float fl;
        float fr;
        float sl;
        float sr;
    };

    class IRsensor
    {
    private:
        LED led;
        const int sampling_count = 16;
        float ir_start_base;
        float ir_wall_base;

        uint16_t dma_f[3];
        uint16_t dma_b[2];

        uint32_t fl[16];
        uint32_t fr[16];
        uint32_t sl[16];
        uint32_t sr[16];

        IR_Value ir_value;
        float bat_vol;

    public:
        IRsensor(float ir_start_base, float ir_wall_base);

        void on_front_led();
        void on_side_led();
        void off_front_led();
        void off_side_led();
        void on_all_led();
        void off_all_led();

        void UI_led_onoff();
        void UI_led_off();

        void StartDMA();
        void UpdateSideValue();
        void UpdateFrontValue();
        void Update();
        IR_Value GetIRSensorData();
        float GetBatteryVoltage();
        void BatteryCheck();
        bool StartInitialize();
    };
}
#endif //  HARDWARE_IR_SENSOR_HPP_