#ifndef UNDERCARRIAGE_KANAYAMA_H_
#define UNDERCARRIAGE_KANAYAMA_H_

#include "main.h"
#include <vector>

namespace undercarriage
{
    class Kanayama
    {
    public:
        Kanayama(const float Kx, const float Ky, const float Ktheta);

        void UpdateRef(const std::vector<float> &ref_pos, const std::vector<float> &ref_vel);
        std::vector<float> CalcInput(const std::vector<float> &cur_pos);
        void Reset();

    private:
        const float Kx;
        const float Ky;
        const float Ktheta;

        float ref_x;
        float ref_y;
        float ref_theta;
        float ref_w;
        float ref_v;
        float x_e;
        float y_e;
        float theta_e;
        std::vector<float> ref_u = {0.0, 0.0};
    };
} // namespace undercarriage

#endif //  UNDERCARRIAGE_KANAYAMA_H_