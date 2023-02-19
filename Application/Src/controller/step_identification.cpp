#include "controller/step_identification.h"

namespace undercarriage
{
    Step_Identification::Step_Identification()
        : u_w(0),
          flag(true),
          index(0)
    {
        output = new float[ref_time];
    }

    void Step_Identification::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Step_Identification::IdenTrans(const ctrl::Pose &cur_vel)
    {
        if (index < ref_time)
        {
            u_v = 1.5;
            output[index] = cur_vel.x;
            InputVelocity(u_v, 0);
            index++;
        }
        else if (index == ref_time)
        {
            motor.Brake();
            flag = false;
        }
    }

    void Step_Identification::InputVelocity(float input_v, float input_w)
    {
        v_left = input_v - input_w;
        v_right = input_v + input_w;
        motor.Drive(v_left, v_right);
    }

    bool Step_Identification::GetFlag()
    {
        return flag;
    }

    void Step_Identification::OutputLog()
    {
        for (int i = 0; i < ref_time; i++)
        {
            printf("%f\n", output[i]);
        }
    }
}