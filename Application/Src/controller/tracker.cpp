/*
 * tracker.cpp
 *
 *  Created on: Jan 27th, 2024
 *      Author: Reiji Terunuma
 */

#include "controller/tracker.h"

namespace undercarriage
{
    void Kanayama::UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a)
    {
        ref_pos = ref_p;
        ref_vel = ref_v;
    }

    ctrl::Pose Kanayama::CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v)
    {
        const float cos_th = cos(cur_pos.th);
        const float sin_th = sin(cur_pos.th);
        error_pos.x = (ref_pos.x - cur_pos.x) * cos_th + (ref_pos.y - cur_pos.y) * sin_th;
        error_pos.y = -(ref_pos.x - cur_pos.x) * sin_th + (ref_pos.y - cur_pos.y) * cos_th;
        error_pos.th = ref_pos.th - cur_pos.th;

        ref_u.x = ref_vel.x * cos(error_pos.th) + Kx * error_pos.x;
        ref_u.th = ref_vel.th + ref_vel.x * 1e-3 * (Ky * error_pos.y + Ktheta * sin(error_pos.th));

        return ref_u;
    }

    void Kanayama::Reset()
    {
        error_pos.clear();
        ref_pos.clear();
        ref_vel.clear();
        ref_u.clear();
    }

    void DynamicFeedback::UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a)
    {
        ref_pos = ref_p;
        cos_th_r = cos(ref_pos.th);
        sin_th_r = sin(ref_pos.th);
        ref_vel.x = ref_v.x * cos_th_r;
        ref_vel.y = ref_v.x * sin_th_r;
        ref_vel.th = ref_v.th;
        ref_acc = ref_a;
    }

    ctrl::Pose DynamicFeedback::CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v)
    {
        const float cos_th = cos(cur_pos.th);
        const float sin_th = sin(cur_pos.th);
        cur_vel.x = cur_v.x * cos_th;
        cur_vel.y = cur_v.x * sin_th;

        const float ux = ref_acc.x + kd * (ref_vel.x - cur_vel.x) + kp * (ref_pos.x - cur_pos.x);
        const float uy = ref_acc.y + kd * (ref_vel.y - cur_vel.y) + kp * (ref_pos.y - cur_pos.y);
        // const float dux = dddx_r + kdx * (ddx_r - ddx) + kpx * (dx_r - dx);
        // const float duy = dddy_r + kdy * (ddy_r - ddy) + kpy * (dy_r - dy);

        const float d_xi = ux * cos_th_r + uy * sin_th_r;
        // const float d_xi = ux * cos_th + uy * sin_th;
        xi += (pre_d_xi + d_xi) * control_period * 0.5;
        pre_d_xi = d_xi;

        if (abs(xi) < xi_threshold)
        {
            // time-varing feedback
            const float v_d = ref_vel.x * cos_th_r + ref_vel.y * sin_th_r;
            const float w_d = ref_vel.th;
            const auto k1 = 2 * zeta * sqrt(w_d * w_d + b * v_d * v_d);
            const auto k2 = b;
            const auto k3 = k1;
            ref_u.x = v_d * cos(cos_th_r - cos_th) + k1 * (cos_th * (ref_pos.x - cur_pos.x) + sin_th * (ref_pos.y - cur_pos.y));
            ref_u.th = w_d + k2 * v_d * sinc(ref_pos.th - cur_pos.th) * (cos_th * (ref_pos.x - cur_pos.x) - sin_th * (ref_pos.y - cur_pos.y)) + k3 * (ref_pos.th - cur_pos.th);
        }
        else
        {
            ref_u.x = xi;
            ref_u.th = (uy * cos_th_r - ux * sin_th_r) / xi;
            // ref_u.th = (uy * cos_th - ux * sin_th) / xi;
        }
        return ref_u;
    }

    void DynamicFeedback::Reset()
    {
        // xi = 0.0;
        pre_d_xi = 0.0;

        ref_pos.clear();
        ref_vel.clear();
        ref_acc.clear();
        ref_u.clear();
        ref_du.clear();
    }

    void TimeVaringFeedback::UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a)
    {
        ref_pos = ref_p;
        cos_th_r = cos(ref_pos.th);
        sin_th_r = sin(ref_pos.th);
        ref_vel = ref_v;
    }

    ctrl::Pose TimeVaringFeedback::CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v)
    {
        const float cos_th = cos(cur_pos.th);
        const float sin_th = sin(cur_pos.th);

        const float v_d = ref_vel.x;
        const float w_d = ref_vel.th;
        const auto k1 = 2 * zeta * sqrt(w_d * w_d + b * v_d * v_d);
        const auto k2 = b;
        const auto k3 = k1;
        ref_u.x = v_d * cos(cos_th_r - cos_th) + k1 * (cos_th * (ref_pos.x - cur_pos.x) + sin_th * (ref_pos.y - cur_pos.y));
        ref_u.th = w_d + k2 * v_d * sinc(ref_pos.th - cur_pos.th) * (cos_th * (ref_pos.x - cur_pos.x) - sin_th * (ref_pos.y - cur_pos.y)) + k3 * (ref_pos.th - cur_pos.th);
        return ref_u;
    }

    void TimeVaringFeedback::Reset()
    {
        ref_pos.clear();
        ref_vel.clear();
        ref_u.clear();
    }

} // namespace undercarriage
