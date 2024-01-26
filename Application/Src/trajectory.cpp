#include "trajectory.h"

namespace trajectory
{
    // Slalom
    void Slalom::ResetTrajectory(int angle, float ref_theta, ctrl::Pose cur_pos)
    {
        switch (slalom_mode)
        {
        case 1:
            v_ref = velocity->v1;
            ss = ss_turn90_1;
            if (angle == 90)
                st = ctrl::slalom::Trajectory(ss);
            else if (angle == -90)
                st = ctrl::slalom::Trajectory(ss, true);

            // if (angle == 90)
            //     ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, 90 - cur_pos.y, ref_theta), 80, 0, 1000 * M_PI, 10 * M_PI, 2.0 * M_PI);
            // else if (angle == -90)
            //     ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, -90 - cur_pos.y, ref_theta), 80, 0, 1000 * M_PI, 10 * M_PI, 2.0 * M_PI);
            // st = ctrl::slalom::Trajectory(ss);
            break;
        case 2:
            v_ref = velocity->v2;
            ss = ss_turn90_2;
            if (angle == 90)
                st = ctrl::slalom::Trajectory(ss);
            else if (angle == -90)
                st = ctrl::slalom::Trajectory(ss, true);

            // if (angle == 90)
            //     ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, 90 - cur_pos.y, ref_theta), 80, 0, 1000 * M_PI, 30 * M_PI, 5.0 * M_PI);
            // else if (angle == -90)
            //     ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, -90 - cur_pos.y, ref_theta), 80, 0, 1000 * M_PI, 30 * M_PI, 5.0 * M_PI);
            // st = ctrl::slalom::Trajectory(ss);
            break;
        default:
            break;
        }
        // printf("v_ref = %f\n", ss.v_ref);
        st.reset(v_ref, 0, 0);
        // t_end = st.getAccelDesigner().t_3() + ss.straight_prev / ss.v_ref;
        t = 0;
        t_end = st.getTimeCurve();
    }

    int Slalom::GetRefSize()
    {
        ref_size = (st.getAccelDesigner().t_3() + ss.straight_prev / ss.v_ref) * 1e+3;
        return ref_size;
    }

    void Slalom::UpdateRef()
    {
        st.update(state, t, Ts, 0);
        ref_pos.x = state.q.x;
        ref_pos.y = state.q.y;
        ref_pos.th = state.q.th;
        ref_vel.x = st.getVelocity();
        ref_vel.y = 0.0;
        ref_vel.th = state.dq.th;
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

    // StaticTrajectoryBase
    void StaticTrajectoryBase::Reset()
    {
        flag = false;
        index = 0;
    }

    void PivotTurn90::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_w[index];
            ref_a = ref_dw[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = true;
        }
    }

    void PivotTurn180::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_w[index];
            ref_a = ref_dw[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = true;
        }
    }

    // M Sequence
    void M_sequence::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_u_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = true;
        }
    }
}