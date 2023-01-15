#include "trajectory.h"

namespace trajectory
{
    // Slalom
    Slalom::Slalom(ctrl::slalom::Shape *ss_turn90_1)
        : ss_turn90_1(ss_turn90_1),
          ss(ss_turn90_1),
          st(ctrl::slalom::Trajectory(*ss)),
          flag_slalom(false)
    {
        ResetTrajectory();
    }

    void Slalom::ResetTrajectory(int angle)
    {
        switch (slalom_mode)
        {
        case 1:
            // if (angle == 90)
            // {
            //     ss = ss_turn90_1;
            //     st = ctrl::slalom::Trajectory(*ss);
            // }
            // else if (angle == -90)
            // {
            //     ss = ss_turn90_1;
            //     st = ctrl::slalom::Trajectory(*ss, true);
            // }
            break;
        // case 2:
        //     if (angle == 90.0)
        //     {
        //         ss = ss_turn90_2;
        //         st = ctrl::slalom::Trajectory(ss);
        //     }
        //     break;
        default:
            break;
        }

        v = ss->v_ref;
        st.reset(v, 0, ss->straight_prev / v);
        const ctrl::AccelDesigner ad(ss->dddth_max, ss->ddth_max, ss->dth_max, 0, 0,
                                     ss->curve.th);
        t_end = st.getAccelDesigner().t_3() + ss->straight_prev / v;
    }

    void Slalom::SetMode(int slalom_mode)
    {
        this->slalom_mode = slalom_mode;
    }

    int Slalom::GetRefSize()
    {
        ref_size = (st.getAccelDesigner().t_3() + ss->straight_prev / v) * 1e+3;
        return ref_size;
    }

    void Slalom::UpdateRef()
    {
        st.update(state, t, Ts, 0);
        ref_pos[0] = state.q.x * 1e-3;
        ref_pos[1] = state.q.y * 1e-3;
        ref_pos[2] = state.q.th;
        ref_vel[0] = v * 1e-3;
        ref_vel[1] = state.dq.th;
        // ref_acc[0] = (state.ddq.x * cos(ref_pos[2]) + state.ddq.y * sin(ref_pos[2])) * 1e-3;
        ref_acc[0] = 0.0;
        ref_acc[1] = state.ddq.th;

        // ref_acc[0] = state.ddq.x * 1e-3;
        // ref_acc[1] = state.ddq.y * 1e-3;

        t += Ts;
        if (t + Ts > t_end)
        {
            flag_slalom = true;
        }
    }

    std::vector<float> Slalom::GetRefPosition()
    {
        return ref_pos;
    }

    std::vector<float> Slalom::GetRefVelocity()
    {
        return ref_vel;
    }

    std::vector<float> Slalom::GetRefAcceleration()
    {
        return ref_acc;
    }

    bool Slalom::Finished()
    {
        return flag_slalom;
    }

    void Slalom::Reset()
    {
        flag_slalom = false;
        t = 0;
    }

    // Acceleration
    Acceleration::Acceleration(const std::array<float, 8> &parameters_start1,
                               const std::array<float, 8> &parameters_forward1,
                               const std::array<float, 8> &parameters_stop1)
        : parameters_start1(parameters_start1),
          parameters_forward1(parameters_forward1),
          parameters_forward_half(parameters_stop1),
          parameters_stop1(parameters_stop1),
          flag_acc(false)
    {
        ad1.reset(parameters_start1);
        ad2.reset(parameters_forward1);
        ad3.reset(parameters_stop1);
        ResetAccCurve(start);
    }

    void Acceleration::ResetAccCurve(const AccType &acc_type)
    {
        this->acc_type = acc_type;
        switch (acc_mode)
        {
        case 1:
            switch (acc_type)
            {
            case start:
                // ad.reset(parameters_start1);
                t_end = ad1.t_end();
                break;
            case forward_half:
                // ad.reset(parameters_forward_half);
                t_end = ad3.t_end();
                break;
            case forward1:
                // ad.reset(parameters_forward1);
                t_end = ad2.t_end();
                break;
            case stop:
                // ad.reset(parameters_stop1);
                t_end = ad3.t_end();
                break;
            default:
                break;
            }
            break;
        // case 2:
        //     break;
        default:
            break;
        }
        // t_end = ad.t_end();
    }

    void Acceleration::SetMode(int acc_mode)
    {
        this->acc_mode = acc_mode;
    }

    int Acceleration::GetRefSize()
    {
        ref_size = ad.t_end() * 1e+3;
        return ref_size;
    }

    void Acceleration::UpdateRef()
    {
        switch (acc_type)
        {
        case start:
            ref_acc = ad1.a(t);
            ref_vel = ad1.v(t);
            ref_pos = ad1.x(t);
            break;
        case forward_half:
            ref_acc = ad3.a(t);
            ref_vel = ad3.v(t);
            ref_pos = ad3.x(t);
            break;
        case forward1:
            ref_acc = ad2.a(t);
            ref_vel = ad2.v(t);
            ref_pos = ad2.x(t);
            break;
        case stop:
            ref_acc = ad3.a(t);
            ref_vel = ad3.v(t);
            ref_pos = ad3.x(t);
            break;
        default:
            break;
        }

        // ref_acc = ad.a(t);
        // ref_vel = ad.v(t);
        // ref_pos = ad.x(t);

        t += Ts;
        if (t > t_end)
        {
            flag_acc = true;
        }
    }

    float Acceleration::GetRefPosition()
    {
        return ref_pos;
    }

    float Acceleration::GetRefVelocity()
    {
        return ref_vel;
    }

    float Acceleration::GetRefAcceleration()
    {
        return ref_acc;
    }

    bool Acceleration::Finished()
    {
        return flag_acc;
    }

    void Acceleration::Reset()
    {
        flag_acc = false;
        t = 0;
    }

    // PivotTurn90
    PivotTurn90::PivotTurn90()
        : index(0),
          flag(false)
    {
        ref_size = GetRefSize();
    }

    void PivotTurn90::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = true;
        }
    }

    int PivotTurn90::GetRefSize()
    {
        return ref_w.size();
    }

    bool PivotTurn90::Finished()
    {
        return flag;
    }

    void PivotTurn90::Reset()
    {
        flag = false;
        index = 0;
    }

    float PivotTurn90::GetRefVelocity()
    {
        return ref;
    }

    // PivotTurn180
    PivotTurn180::PivotTurn180()
        : index(0),
          flag(false)
    {
        ref_size = GetRefSize();
    }

    void PivotTurn180::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = true;
        }
    }

    int PivotTurn180::GetRefSize()
    {
        return ref_w.size();
    }

    bool PivotTurn180::Finished()
    {
        return flag;
    }

    void PivotTurn180::Reset()
    {
        flag = false;
        index = 0;
    }

    float PivotTurn180::GetRefVelocity()
    {
        return ref;
    }

    // M Sequence
    M_sequence::M_sequence()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void M_sequence::ResetTrajectoryIndex()
    {
        index = 0;
    }

    int M_sequence::GetRefSize()
    {
        return ref_u_w.size();
    }

    void M_sequence::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_u_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    float M_sequence::GetRef()
    {
        return ref;
    }

    bool M_sequence::GetFlag()
    {
        return flag;
    }
}