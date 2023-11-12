#include "controller/kanayama.h"

namespace undercarriage
{
    Kanayama::Kanayama(const float Kx, const float Ky, const float Ktheta)
        : Kx(Kx),
          Ky(Ky),
          Ktheta(Ktheta) {}

    void Kanayama::UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v)
    {
        // ref_pos.x = ref_p.x;
        // ref_pos.y = ref_p.y;
        // ref_pos.th = ref_p.th;
        // ref_vel.x = ref_v.x;
        // ref_vel.y = ref_v.y;
        // ref_vel.th = ref_v.th;

        ref_pos = ref_p;
        ref_vel = ref_v;
    }

    ctrl::Pose Kanayama::CalcInput(const ctrl::Pose &cur_pos)
    {
        error_pos.x = (ref_pos.x - cur_pos.x) * cos(cur_pos.th) + (ref_pos.y - cur_pos.y) * sin(cur_pos.th);
        error_pos.y = -(ref_pos.x - cur_pos.x) * sin(cur_pos.th) + (ref_pos.y - cur_pos.y) * cos(cur_pos.th);
        error_pos.th = ref_pos.th - cur_pos.th;

        ref_u.x = ref_vel.x * cos(error_pos.th) + Kx * error_pos.x;
        ref_u.th = ref_vel.th + ref_vel.x * (Ky * error_pos.y + Ktheta * sin(error_pos.th));

        return ref_u;
    }

    void Kanayama::Reset()
    {
        error_pos.clear();
        ref_pos.clear();
        ref_vel.clear();
        ref_u.clear();
    }
}