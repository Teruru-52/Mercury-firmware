#include "controller/m_identification.h"

namespace undercarriage
{
    Identification::Identification()
        : u_w(0),
          flag(true),
          index(0),
          index_log(0)
    {
        ref_size = m_sequence.GetRefSize() * 10;
        input = new float[ref_size];
        output = new float[ref_size];
    }

    void Identification::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Identification::UpdateRef()
    {
        m_sequence.UpdateRef();
        u_w = m_sequence.GetRef();
    }

    void Identification::IdenRotate(const ctrl::Pose &cur_vel)
    {
        if (m_sequence.GetFlag())
        {
            if (index % 200 == 0)
            {
                UpdateRef();
            }
            if (index % 20 == 0)
            {
                input[index_log] = u_w;
                output[index_log] = cur_vel.th;
                index_log++;
            }
            InputVelocity(0, u_w);
            index++;
        }
        else
        {
            motor.Brake();
            m_sequence.ResetTrajectoryIndex();
            flag = false;
        }
    }

    void Identification::InputVelocity(float input_v, float input_w)
    {
        v_left = input_v - input_w;
        v_right = input_v + input_w;
        motor.Drive(v_left, v_right);
    }

    bool Identification::GetFlag()
    {
        return flag;
    }

    void Identification::OutputLog()
    {
        for (int i = 0; i < ref_size; i++)
        {
            printf("%f, %f\n", input[i], output[i]);
        }
    }
}