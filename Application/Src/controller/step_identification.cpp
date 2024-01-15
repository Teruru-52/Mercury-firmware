#include "controller/step_identification.h"

namespace undercarriage
{
    Step_Identification::Step_Identification()
        : u_w(0),
          flag(false),
          index(0)
    {
        output = new float[ref_time];
    }

    float Step_Identification::GetTransInput(const ctrl::Pose &cur_vel)
    {
        if (index < ref_time)
        {
            u_v = 1.5;
            output[index] = cur_vel.x;
            index++;
        }
        else if (index == ref_time)
        {
            u_v = 0;
            flag = true;
        }
        return u_v;
    }

    void Step_Identification::OutputLog()
    {
        for (int i = 0; i < ref_time; i++)
            printf("%f\n", output[i]);
    }
}