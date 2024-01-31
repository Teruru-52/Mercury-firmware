/*
 * identification.cpp
 *
 *  Refactored on: Jan 27th, 2024
 *      Author: Reiji Terunuma
 */

#include "controller/identification.h"

namespace undercarriage
{
    // M_Identification
    M_Identification::M_Identification()
        : index_log(0)
    {
        ref_size = m_sequence.GetRefSize() * 10;
        input = new float[ref_size];
        output = new float[ref_size];
    }

    void M_Identification::UpdateRef()
    {
        m_sequence.UpdateRef();
        ref_size = m_sequence.GetRefSize();
        u = m_sequence.GetRefVoltage();
    }

    float M_Identification::GetInput(const float cur_vel)
    {
        if (m_sequence.Finished())
        {
            u = 0;
            m_sequence.Reset();
            flag = true;
        }
        else
        {
            if (index % 200 == 0)
                UpdateRef();
            if (index % 20 == 0)
            {
                input[index_log] = u;
                output[index_log] = cur_vel;
                index_log++;
            }
            index++;
        }
        return u;
    }

    void M_Identification::OutputLog()
    {
        for (int i = 0; i < ref_size; i++)
            printf("%f, %f\n", input[i], output[i]);
    }

    // Step_Identification
    float Step_Identification::GetInput(const float cur_vel)
    {
        if (index < ref_time)
        {
            u = 1.5;
            output[index] = cur_vel;
            index++;
        }
        else if (index == ref_time)
        {
            u = 0;
            flag = true;
        }
        return u;
    }

    void Step_Identification::OutputLog()
    {
        for (int i = 0; i < ref_time; i++)
            printf("%f\n", output[i]);
    }

} // namespace undercarriage