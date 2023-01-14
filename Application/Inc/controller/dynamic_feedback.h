#ifndef DYNAMIC_FEEDBACK_H_
#define DYNAMIC_FEEDBACK_H_

#include "main.h"
#include <vector>

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
        void UpdateRef(const std::vector<float> &ref_pos, const std::vector<float> &ref_vel, const std::vector<float> &ref_acc);
        std::vector<float> CalcInput(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void Reset();

    private:
        const float kp1;
        const float kd1;
        const float kp2;
        const float kd2;
        const float control_period;

        float ref_x;
        float ref_y;
        float ref_vx;
        float ref_vy;
        float ref_ax;
        float ref_ay;

        float xi = 0.0;
        float dxi;
        float pre_dxi = 0.0;

        float u_x;
        float u_y;
        float u_v;
        float u_w;
        std::vector<float> ref_u = {0.0, 0.0};
    };

} // namespace undercarriage

#endif // DYNAMIC_FEEDBACK_H_