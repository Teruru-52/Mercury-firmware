#ifndef UNDERCARRIAGE_KANAYAMA_H_
#define UNDERCARRIAGE_KANAYAMA_H_

#include "main.h"
#include "pose.h"

namespace undercarriage
{
    class Kanayama
    {
    public:
        Kanayama(const float Kx, const float Ky, const float Ktheta);

        void UpdateRef(const ctrl::Pose &ref_pos, const ctrl::Pose &ref_vel);
        ctrl::Pose CalcInput(const ctrl::Pose &cur_pos);
        void Reset();

    private:
        const float Kx;
        const float Ky;
        const float Ktheta;

        ctrl::Pose error_pos{0, 0, 0};
        ctrl::Pose ref_pos{0, 0, 0};
        ctrl::Pose ref_vel{0, 0, 0};
        ctrl::Pose ref_u{0, 0, 0};
    };
} // namespace undercarriage

#endif //  UNDERCARRIAGE_KANAYAMA_H_