#include "controller/m_identification.h"

namespace undercarriage
{
    Identification::Identification()
        : u_w(0),
          flag(false),
          index(0),
          index_log(0)
    {
        ref_size = m_sequence.GetRefSize() * 10;
        input = new float[ref_size];
        output = new float[ref_size];
    }

    void Identification::UpdateRef()
    {
        m_sequence.UpdateRef();
        u_w = m_sequence.GetRefVoltage() * 0.5;
    }

    float Identification::GetRotInput(const ctrl::Pose &cur_vel)
    {
        if (m_sequence.Finished())
        {
            u_w = 0;
            m_sequence.ResetTrajectoryIndex();
            flag = true;
        }
        else
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
            index++;
        }
        return u_w;
    }

    void Identification::OutputLog()
    {
        for (int i = 0; i < ref_size; i++)
            printf("%f, %f\n", input[i], output[i]);
    }
}