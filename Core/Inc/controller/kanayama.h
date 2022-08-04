#ifndef UNDERCARRIAGE_KANAYAMA_HPP_
#define UNDERCARRIAGE_KANAYAMA_HPP_

#include "main.h"
#include <vector>
#include "trajectory.h"

namespace undercarriage
{
    class Kanayama
    {
    public:
        Kanayama(float Kx, float Ky, float Ktheta);

        void UpdateRef();
        void UpdateRef2();
        void ResetTrajectoryIndex();
        std::vector<float> CalcInput(std::vector<float> cur_pos);
        bool GetFlag();

    private:
        trajectory::TurnLeft90 turnleft90;
        float Kx;
        float Ky;
        float Ktheta;
        std::vector<float> ref;
        float ref_x;
        float ref_y;
        float ref_theta;
        float ref_w;
        const float ref_v = 0.5064989;
        float x_e;
        float y_e;
        float theta_e;
        std::vector<float> ref_u;
        bool flag;
    };
} // namespace undercarriage

#endif //  UNDERCARRIAGE_KANAYAMA_HPP_