#ifndef DYNAMIC_FEEDBACK_H_
#define DYNAMIC_FEEDBACK_H_

#include "main.h"
#include "pose.h"

namespace undercarriage
{
    class Dynamic_Feedback
    {
    public:
        Dynamic_Feedback(const float kp1,
                         const float kd1,
                         const float kp2,
                         const float kd2,
                         const float control_period);
        void UpdateRef(const ctrl::Pose &ref_pos, const ctrl::Pose &ref_vel, const ctrl::Pose &ref_acc);
        ctrl::Pose CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_vel);
        void Reset();

    private:
        const float kp1;
        const float kd1;
        const float kp2;
        const float kd2;
        const float control_period;

        float xi = 0.0;
        float dxi;
        float pre_dxi = 0.0;

        float u_x;
        float u_y;
        float u_v;
        float u_w;
        ctrl::Pose ref_pos{0, 0, 0};
        ctrl::Pose ref_vel{0, 0, 0};
        ctrl::Pose ref_acc{0, 0, 0};
        ctrl::Pose ref_u{0, 0, 0};
    };

} // namespace undercarriage

#endif // DYNAMIC_FEEDBACK_H_