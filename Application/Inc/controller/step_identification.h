#ifndef STEP_INDENTIFICATION_HPP_
#define STEP_INDENTIFICATION_HPP_

#include "main.h"
#include "hardware/motor.h"
#include "pose.h"

namespace undercarriage
{
    class Step_Identification
    {
    public:
        Step_Identification();
        void UpdateBatteryVoltage(float bat_vol);
        void IdenTrans(const ctrl::Pose &cur_vel);
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        void OutputLog();

    private:
        hardware::Motor motor;

        float v_left;
        float v_right;
        float u_v;
        float u_w;
        bool flag;
        int index;
        int ref_time = 3000; // [ms]
        float *output;
    };
} // namespace undercarriage

#endif //  STEP_INDENTIFICATION_HPP_