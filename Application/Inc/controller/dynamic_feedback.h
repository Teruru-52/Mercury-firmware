#ifndef DYNAMIC_FEEDBACK_H_
#define DYNAMIC_FEEDBACK_H_

#include "main.h"
#include "pose.h"

namespace undercarriage
{
    class Dynamic_Feedback
    {
    public:
        Dynamic_Feedback(const float kpx,
                         const float kdx,
                         const float kpy,
                         const float kdy,
                         const float control_period);
        void UpdateRef(const ctrl::Pose &ref_pos, const ctrl::Pose &ref_vel, const ctrl::Pose &ref_acc);
        ctrl::Pose CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_vel);
        ctrl::Pose GetInputAcc() { return ref_du; }
        void Reset();

    private:
        const float kpx;
        const float kdx;
        const float kpy;
        const float kdy;
        const float control_period;

        const float zeta = 1.0f;
        const float omega_n = 15.0f;
        const float low_zeta = 1.0f; /*< zeta \in [0,1] */
        const float low_b = 1e-3f;   /*< b > 0 */

        float xi = 0.0;
        float d_xi;
        float pre_d_xi = 0.0;
        static constexpr const float xi_threshold = 0.15f;

        float cos_th_r;
        float sin_th_r;
        ctrl::Pose ref_pos{0, 0, 0};
        ctrl::Pose ref_vel{0, 0, 0};
        ctrl::Pose ref_acc{0, 0, 0};
        ctrl::Pose ref_u{0, 0, 0};
        ctrl::Pose ref_du{0, 0, 0};
    };

} // namespace undercarriage

#endif // DYNAMIC_FEEDBACK_H_