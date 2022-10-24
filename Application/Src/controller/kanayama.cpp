#include "../../Inc/controller/kanayama.h"

namespace undercarriage
{
    Kanayama::Kanayama(float Kx, float Ky, float Ktheta)
        : Kx(Kx),
          Ky(Ky),
          Ktheta(Ktheta),
          ref({0.0, 0.0, 0.0, 0.0}),
          ref_u({0.0, 0.0}),
          flag(true) {}

    void Kanayama::UpdateRef()
    {
        if (turnleft90.GetFlag())
        {
            turnleft90.UpdateRef();
            ref = turnleft90.GetRef();
            ref_x = ref[0];
            ref_y = ref[1];
            ref_theta = ref[2];
            ref_w = ref[3];
        }
        else
        {
            flag = false;
        }
    }

    void Kanayama::UpdateRef2()
    {
        if (turnleft90.GetFlag())
        {
            turnleft90.UpdateRef();
            ref = turnleft90.GetRef();
            ref_x = ref[0];
            ref_y = -ref[1];
            ref_theta = -ref[2];
            ref_w = -ref[3];
        }
        else
        {
            flag = false;
        }
    }

    void Kanayama::Reset()
    {
        turnleft90.ResetTrajectoryIndex();
        turnleft90.ResetFlag();
        flag = true;
    }

    std::vector<float> Kanayama::CalcInput(const std::vector<float> &cur_pos)
    {
        x_e = (ref_x - cur_pos[0]) * cos(cur_pos[2]) + (ref_y - cur_pos[1]) * sin(cur_pos[2]);
        y_e = -(ref_x - cur_pos[0]) * sin(cur_pos[2]) + (ref_y - cur_pos[1]) * cos(cur_pos[2]);
        theta_e = ref_theta - cur_pos[2];

        ref_u[0] = ref_v * cos(theta_e) + Kx * x_e;
        ref_u[1] = ref_w + ref_v * (Ky * y_e + Ktheta * sin(theta_e));

        return ref_u;
    }

    bool Kanayama::GetFlag()
    {
        return flag;
    }
}