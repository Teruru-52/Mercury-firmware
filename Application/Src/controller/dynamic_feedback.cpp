#include "controller/dynamic_feedback.h"

namespace undercarriage
{
    Dynamic_Feedback::Dynamic_Feedback(const float kp1,
                                       const float kd1,
                                       const float kp2,
                                       const float kd2,
                                       const float control_period)
        : kp1(kp1),
          kd1(kd1),
          kp2(kp2),
          kd2(kd2),
          control_period(control_period) {}

    void Dynamic_Feedback::UpdateRef(const std::vector<float> &ref_pos, const std::vector<float> &ref_vel, const std::vector<float> &ref_acc)
    {
        ref_x = ref_pos[0];
        ref_y = ref_pos[1];
        ref_vx = ref_vel[0] * cos(ref_pos[2]);
        ref_vy = ref_vel[0] * sin(ref_pos[2]);
        // ref_ax = ref_acc[0] * cos(ref_pos[2]) - ref_vel[1] * ref_vy;
        // ref_ay = ref_acc[0] * sin(ref_pos[2]) + ref_vel[1] * ref_vx;
        // ref_ax = ref_acc[0];
        // ref_ay = ref_acc[1];
        ref_ax = 0.0;
        ref_ay = 0.0;
    }

    std::vector<float> Dynamic_Feedback::CalcInput(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        u_x = ref_ax + kp1 * (ref_x - cur_pos[0]) + kd1 * (ref_vx - cur_vel[0] * cos(cur_pos[2]));
        u_y = ref_ay + kp2 * (ref_y - cur_pos[1]) + kd2 * (ref_vy - cur_vel[0] * sin(cur_pos[2]));

        dxi = u_x * cos(cur_pos[2]) + u_y * sin(cur_pos[2]);
        xi += (pre_dxi + dxi) * control_period * 0.5;
        u_v = xi;
        u_w = (u_y * cos(cur_pos[2]) - u_x * sin(cur_pos[2])) / xi;

        pre_dxi = dxi;

        ref_u[0] = u_v;
        ref_u[1] = u_w;
        return ref_u;
    }

    void Dynamic_Feedback::Reset()
    {
        ref_x = 0.0;
        ref_y = 0.0;
        ref_vx = 0.0;
        ref_vy = 0.0;
        ref_ax = 0.0;
        ref_ay = 0.0;

        xi = 0.0;
        pre_dxi = 0.0;

        u_x = 0.0;
        u_y = 0.0;
        u_v = 0.0;
        u_w = 0.0;
    }

} // namespace undercarriage