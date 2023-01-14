#include "controller/kanayama.h"

namespace undercarriage
{
    Kanayama::Kanayama(const float Kx, const float Ky, const float Ktheta)
        : Kx(Kx),
          Ky(Ky),
          Ktheta(Ktheta) {}

    void Kanayama::UpdateRef(const std::vector<float> &ref_pos, const std::vector<float> &ref_vel)
    {
        ref_x = ref_pos[0];
        ref_y = ref_pos[1];
        ref_theta = ref_pos[2];
        ref_v = ref_vel[0];
        ref_w = ref_vel[1];
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

    void Kanayama::Reset()
    {
        ref_x = 0.0;
        ref_y = 0.0;
        ref_theta = 0.0;
        ref_v = 0.0;
        ref_w = 0.0;

        x_e = 0.0;
        y_e = 0.0;
        theta_e = 0.0;
    }
}