#include "controller/dynamic_feedback.h"

namespace undercarriage
{
    Dynamic_Feedback::Dynamic_Feedback(const float kpx,
                                       const float kdx,
                                       const float kpy,
                                       const float kdy,
                                       const float control_period)
        : kpx(kpx),
          kdx(kdx),
          kpy(kpy),
          kdy(kdy),
          control_period(control_period) {}

    void Dynamic_Feedback::UpdateRef(const ctrl::Pose &ref_pos, const ctrl::Pose &ref_vel, const ctrl::Pose &ref_acc)
    {
        cos_th_r = cos(ref_pos.th);
        sin_th_r = sin(ref_pos.th);
        this->ref_pos = ref_pos;
        this->ref_vel.x = ref_vel.x * cos_th_r;
        this->ref_vel.y = ref_vel.x * sin_th_r;
        this->ref_acc = ref_acc;
    }

    ctrl::Pose Dynamic_Feedback::CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_vel)
    {
        const float kpx = omega_n * omega_n;
        const float kdx = 2 * zeta * omega_n;
        const float kpy = kpx;
        const float kdy = kdx;

        const float theta = cur_pos.th;
        const float cos_th = cos(theta);
        const float sin_th = sin(theta);
        const float ux = ref_acc.x + kdx * (ref_vel.x - cur_vel.x) + kpx * (ref_pos.x - cur_pos.x);
        const float uy = ref_acc.y + kdy * (ref_vel.y - cur_vel.y) + kpy * (ref_pos.y - cur_pos.y);
        // const float dux = dddx_r + kdx * (ddx_r - ddx) + kpx * (dx_r - dx);
        // const float duy = dddy_r + kdy * (ddy_r - ddy) + kpy * (dy_r - dy);
        const float d_xi = ux * cos_th_r + uy * sin_th_r;
        xi += (pre_d_xi + d_xi) * control_period * 0.5;
        pre_d_xi = d_xi;

        if (abs(xi) < xi_threshold)
        {
            const float v_d = ref_vel.x * cos_th_r + ref_vel.y * sin_th_r;
            const float w_d = ref_vel.th;
            const auto k1 = 2 * low_zeta * sqrt(w_d * w_d + low_b * v_d * v_d);
            const auto k2 = low_b;
            const auto k3 = k1;
            ref_u.x = v_d * cos(cos_th_r - cos_th) + k1 * (cos_th * (ref_pos.x - cur_pos.x) + sin_th * (ref_pos.y - cur_pos.y));
            ref_u.th = w_d + k2 * v_d * sin(ref_pos.th - cur_pos.th) / (ref_pos.th - cur_pos.th) * (-sin_th * (ref_pos.x - cur_pos.x) + cos_th * (ref_pos.y - cur_pos.y)) + k3 * (ref_pos.th - cur_pos.th);
        }
        else
        {
            ref_u.x = xi;
            ref_u.th = (uy * cos(cur_pos.th) - ux * sin(cur_pos.th)) / xi;
        }
        return ref_u;
    }

    void Dynamic_Feedback::Reset()
    {
        xi = 0.0;
        pre_d_xi = 0.0;

        ref_pos.clear();
        ref_vel.clear();
        ref_acc.clear();
        ref_u.clear();
        ref_du.clear();
    }

} // namespace undercarriage