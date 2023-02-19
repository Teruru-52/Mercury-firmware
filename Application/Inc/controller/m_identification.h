#ifndef INDENTIFICATION_HPP_
#define INDENTIFICATION_HPP_

#include "main.h"
#include "hardware/motor.h"
#include "trajectory.h"
#include "pose.h"

namespace undercarriage
{
    class Identification
    {
    public:
        Identification();
        void UpdateRef();
        void UpdateBatteryVoltage(float bat_vol);
        void IdenRotate(const ctrl::Pose &cur_vel);
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        void OutputLog();
        trajectory::M_sequence m_sequence;

    private:
        hardware::Motor motor;

        float v_left;
        float v_right;
        float u_w;
        bool flag;
        int index;
        int index_log;
        int ref_size;
        float *input;
        float *output;
    };
} // namespace undercarriage

#endif //  INDENTIFICATION_HPP_