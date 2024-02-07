/**
 * @file tracker.h
 * @date Jan 27th, 2024
 * @author Teruru-52
 */

#ifndef TRACKER_H_
#define TRACKER_H_

#include "main.h"
#include "pose.h"

namespace undercarriage
{
    class TrackerBase
    {
    protected:
        float cos_th_r;
        float sin_th_r;
        ctrl::Pose ref_pos{0, 0, 0};
        ctrl::Pose ref_vel{0, 0, 0};
        ctrl::Pose ref_u{0, 0, 0};

    public:
        explicit TrackerBase() {}
        static constexpr float sinc(const float x)
        {
            const auto xx = x * x;
            const auto xxxx = xx * xx;
            return xxxx * xxxx / 362880 - xxxx * xx / 5040 + xxxx / 120 - xx / 6 + 1;
        }
        virtual void SetXi(const float xi_ini = 0) = 0;
        virtual void UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a) = 0;
        virtual ctrl::Pose CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v) = 0;
        virtual void Reset() = 0;
        virtual ~TrackerBase() {}
    };

    class Kanayama : public TrackerBase
    {
    private:
        const float Kx;
        const float Ky;
        const float Ktheta;

        ctrl::Pose error_pos{0, 0, 0};

    public:
        explicit Kanayama(const float Kx, const float Ky, const float Ktheta)
            : Kx(Kx), Ky(Ky), Ktheta(Ktheta) {}
        void SetXi(const float xi_ini = 0) override {}
        void UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a) override;
        ctrl::Pose CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v) override;
        void Reset() override;
    };

    class DynamicFeedback : public TrackerBase
    {
    private:
        const float kp;
        const float kd;
        const float control_period;

        const float zeta = 1.0f; /*< zeta \in [0,1] */
        const float b = 1e-3f;   /*< b > 0 */

        float xi = 0.0;
        float pre_d_xi = 0.0;
        static constexpr const float xi_threshold = 50.0f;

        ctrl::Pose cur_vel{0, 0, 0};
        ctrl::Pose ref_acc{0, 0, 0};
        ctrl::Pose ref_du{0, 0, 0};

    public:
        explicit DynamicFeedback(const float omega_n,
                                 const float zeta,
                                 const float control_period)
            : kp(omega_n * omega_n), kd(2.0 * zeta * omega_n), control_period(control_period) {}
        void SetXi(const float xi_ini = 0) override { xi = xi_ini; }
        void UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a) override;
        ctrl::Pose CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v) override;
        void Reset() override;
    };

    class TimeVaryingFeedback : public TrackerBase
    {
    private:
        const float zeta; /*< zeta \in [0,1] */
        const float b;    /*< b > 0 */

        float cos_th_r;
        float sin_th_r;

    public:
        explicit TimeVaryingFeedback(const float zeta, const float b)
            : zeta(zeta), b(b) {}
        void SetXi(const float xi_ini = 0) override {}
        void UpdateRef(const ctrl::Pose &ref_p, const ctrl::Pose &ref_v, const ctrl::Pose &ref_a) override;
        ctrl::Pose CalcInput(const ctrl::Pose &cur_pos, const ctrl::Pose &cur_v) override;
        void Reset() override;
    };

} // namespace undercarriage

#endif // TRACKER_H_