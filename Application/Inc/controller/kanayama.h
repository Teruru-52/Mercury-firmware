#ifndef UNDERCARRIAGE_KANAYAMA_H_
#define UNDERCARRIAGE_KANAYAMA_H_

#include "main.h"
#include <vector>

namespace undercarriage
{
    class Kanayama
    {
    public:
        Kanayama(float Kx, float Ky, float Ktheta);

        void UpdateRef(const std::vector<float> &ref_pos, const std::vector<float> &ref_vel);
        void Reset();
        std::vector<float> CalcInput(const std::vector<float> &cur_pos);
        bool GetFlag();

    private:
        float Kx;
        float Ky;
        float Ktheta;
        std::vector<float> ref;
        float ref_x;
        float ref_y;
        float ref_theta;
        float ref_w;
        float ref_v = 0.5064989;
        float x_e;
        float y_e;
        float theta_e;
        std::vector<float> ref_u;
        bool flag;
    };
} // namespace undercarriage

#endif //  UNDERCARRIAGE_KANAYAMA_H_