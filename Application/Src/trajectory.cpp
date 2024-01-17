#include "trajectory.h"

namespace trajectory
{
    // Slalom
    void Slalom::ResetTrajectory(int angle)
    {
        switch (slalom_mode)
        {
        case 1:
            ss = ss_turn90_1;
            if (angle == 90)
                st = ctrl::slalom::Trajectory(ss);
            else if (angle == -90)
                st = ctrl::slalom::Trajectory(ss, true);
            break;
        case 2:
            ss = ss_turn90_2;
            if (angle == 90)
                st = ctrl::slalom::Trajectory(ss);
            else if (angle == -90)
                st = ctrl::slalom::Trajectory(ss, true);
            break;
        default:
            break;
        }

        v = ss.v_ref;
        // printf("v_ref = %f\n", v);
    }

    void Slalom::SetInitialTime(bool flag_front_wall)
    {
        if (flag_front_wall)
        {
            t = ss.straight_prev / v;
            st.reset(v, 0, t);
            const ctrl::AccelDesigner ad(ss.dddth_max, ss.ddth_max, ss.dth_max, 0, 0,
                                         ss.curve.th);
            t_end = st.getAccelDesigner().t_3() + ss.straight_prev / v;
        }
        else
        {
            t = 0.0;
            st.reset(v, 0, ss.straight_prev / v);
            const ctrl::AccelDesigner ad(ss.dddth_max, ss.ddth_max, ss.dth_max, 0, 0,
                                         ss.curve.th);
            t_end = st.getAccelDesigner().t_3() + ss.straight_prev / v;
        }
        // printf("t_end = %f\n", t_end);
    }

    int Slalom::GetRefSize()
    {
        ref_size = (st.getAccelDesigner().t_3() + ss.straight_prev / v) * 1e+3;
        return ref_size;
    }

    void Slalom::UpdateRef()
    {
        st.update(state, t, Ts, 0);
        ref_pos.x = state.q.x;
        ref_pos.y = state.q.y;
        ref_pos.th = state.q.th;
        ref_vel.x = v;
        ref_vel.y = 0.0;
        ref_vel.th = state.dq.th;
        // ref_acc.x = (state.ddq.x * cos(ref_pos.th) + state.ddq.y * sin(ref_pos.th)) * 1e-3;
        // ref_acc.y = 0.0;
        ref_acc.x = state.ddq.x;
        ref_acc.y = state.ddq.y;
        ref_acc.th = state.ddq.th;

        t += Ts;
        if ((t > t_end * 0.8) && (!flag_time))
        {
            flag_read_side_wall = true;
            flag_time = true;
        }
        if (t + Ts > t_end)
        {
            flag_slalom = true;
        }
    }

    void Slalom::Reset()
    {
        flag_slalom = false;
        flag_time = false;
        t = 0;
        state.q.x = state.q.y = 0.0;
    }

    // Acceleration
    Acceleration::Acceleration(Velocity *velocity)
        : velocity(velocity),
          flag_acc(false)
    {
        ResetAccCurve(start);
        // param_stop0 = {10000, 1500, 500, 0, 0, FORWARD_LENGTH_HALF, 0, 0};
        // param_start0 = {10000, 1500, 500, 0, 0, FORWARD_LENGTH_START, 0, 0};
        // param_forward0 = {10000, 1500, 500, 0, 0, FORWARD_LENGTH, 0, 0};
        param_stop1 = {10000, 1500, 300, velocity->v1, 0, FORWARD_LENGTH_HALF, 0, 0};
        param_start1 = {10000, 1500, 300, 0, velocity->v1, FORWARD_LENGTH_START, 0, 0};
        param_start_half1 = {10000, 1500, 300, 0, velocity->v1, FORWARD_LENGTH_HALF, 0, 0};
        // param_forward1 = {10000, 1500, 500, velocity->v1, velocity->v1, FORWARD_LENGTH, 0, 0};
        param_stop2 = {20000, 10000.0, 7000, velocity->v2, 0, FORWARD_LENGTH_HALF, 0, 0};
        param_start2 = {20000, 10000.0, 7000, 0, velocity->v2, FORWARD_LENGTH_START, 0, 0};
        param_start_half2 = {20000, 10000.0, 7000, 0, velocity->v2, FORWARD_LENGTH_HALF, 0, 0};
        // param_forward2 = {10000, 1500, 500, velocity->v2, velocity->v2, FORWARD_LENGTH, 0, 0};
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
                ad.reset(param_start1);
                break;
            case start_half:
                ad.reset(param_start_half1);
                break;
            case stop:
                ad.reset(param_stop1);
                break;
            default:
                break;
            }
            break;
        case 2:
            switch (acc_type)
            {
            case start:
                ad.reset(param_start2);
                break;
            case start_half:
                ad.reset(param_start_half2);
                break;
            case stop:
                ad.reset(param_stop2);
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        t_end = ad.t_end();
    }

    int Acceleration::GetRefSize()
    {
        ref_size = ad.t_end() * 1e+3;
        return ref_size;
    }

    void Acceleration::UpdateRef()
    {
        ref_acc = ad.a(t);
        ref_vel = ad.v(t);
        ref_pos = ad.x(t);

        t += Ts;
        if (acc_type != stop)
        {
            if ((t > t_end * 0.8) && (!flag_time))
            {
                flag_read_side_wall = true;
                flag_time = true;
            }
        }
        if (t > t_end)
            flag_acc = true;
    }

    void Acceleration::Reset()
    {
        flag_read_side_wall = false;
        flag_time = false;
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

    void PivotTurn90::Reset()
    {
        flag = false;
        index = 0;
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

    void PivotTurn180::Reset()
    {
        flag = false;
        index = 0;
    }

    // M Sequence
    M_sequence::M_sequence()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
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
}