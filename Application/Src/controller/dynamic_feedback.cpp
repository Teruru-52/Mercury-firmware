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

    void Dynamic_Feedback::UpdateRef(const ctrl::Pose &ref_pos, const ctrl::Pose &ref_vel, const ctrl::Pose &ref_acc)
    {
        this->ref_pos = ref_pos;
        this->ref_vel = ref_vel;
        this->ref_acc = ref_acc;

        // ref_vel.y = ref_vel.x * sin(ref_pos.th);
        // ref_vel.x = ref_vel.x * cos(ref_pos.th);
        // ref_ax = ref_acc.x * cos(ref_pos.th) - ref_vel.th * ref_vel.y;
        // ref_ay = ref_acc.x * sin(ref_pos.th) + ref_vel.th * ref_vel.x;
        // ref_ax = ref_acc.x;
        // ref_ay = ref_acc.y;
        // ref_acc.x = 0.0;
        // ref_acc.y = 0.0;
    }

    ctrl::Pose Dynamic_Feedback::CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_vel)
    {
        u_x = ref_acc.x + kp1 * (ref_vel.x - cur_pos.x) + kd1 * (ref_vel.x - cur_vel.x * cos(cur_pos.th));
        u_y = ref_acc.y + kp2 * (ref_vel.y - cur_pos.y) + kd2 * (ref_vel.y - cur_vel.y * sin(cur_pos.th));

        dxi = u_x * cos(cur_pos.th) + u_y * sin(cur_pos.th);
        xi += (pre_dxi + dxi) * control_period * 0.5;
        u_v = xi;
        u_w = (u_y * cos(cur_pos.th) - u_x * sin(cur_pos.th)) / xi;

        pre_dxi = dxi;

        ref_u.x = u_v;
        ref_u.th = u_w;
        return ref_u;
    }

    void Dynamic_Feedback::Reset()
    {
        xi = 0.0;
        pre_dxi = 0.0;

        u_x = 0.0;
        u_y = 0.0;
        u_v = 0.0;
        u_w = 0.0;

        ref_pos.clear();
        ref_vel.clear();
        ref_acc.clear();
    }

} // namespace undercarriage