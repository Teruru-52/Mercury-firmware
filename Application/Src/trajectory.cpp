/**
 * @file trajectory.cpp
 * @author Reiji Terunuma
 */

#include "trajectory.h"

namespace trajectory
{
    // Slalom
    void Slalom::ResetTrajectory(int angle, float ref_theta, ctrl::Pose cur_pos)
    {
        switch (trj_mode)
        {
        case 1:
            v_ref = velocity->v1;
            if (angle == 90)
                ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, 90, ref_theta), 80, 0, params->run1.j_max, params->run1.a_max, params->run1.v_max);
            // st = ctrl::slalom::Trajectory(ss_turn90);
            else if (angle == -90)
                ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, -90, ref_theta), -80, 0, params->run1.j_max, params->run1.a_max, params->run1.v_max);
            // st = ctrl::slalom::Trajectory(ss_turn90, true);
            else if (angle == 45)
                ss = ctrl::slalom::Shape(ctrl::Pose(90, 90, M_PI * 0.25f), 0, 0, params->run1.j_max, params->run1.a_max, params->run1.v_max);
            else if (angle == -45)
                ss = ctrl::slalom::Shape(ctrl::Pose(90, -90, -M_PI * 0.25f), 0, 0, params->run1.j_max, params->run1.a_max, params->run1.v_max);
            st = ctrl::slalom::Trajectory(ss);
            break;
        case 2:
            v_ref = velocity->v2;

            if (angle == 90)
                ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, 90, ref_theta), 80, 0, params->run2.j_max, params->run2.a_max, params->run2.v_max);
            else if (angle == -90)
                ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, -90, ref_theta), -80, 0, params->run2.j_max, params->run2.a_max, params->run2.v_max);
            break;
        default:
            break;
        }
        // printf("v_ref = %f\n", ss.v_ref);
        // st = ctrl::slalom::Trajectory(ss);
        st.reset(v_ref, 0, 0);
        // t_end = st.getAccelDesigner().t_3() + ss.straight_prev / ss.v_ref;
        t_end = st.getTimeCurve();
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
            flag_trj = true;
    }

    void Slalom::Reset()
    {
        flag_trj = false;
        flag_time = false;
        t = 0;
    }

    // Acceleration
    void Acceleration::ResetTrajectory(const AccType &acc_type, float cur_vel, uint8_t num_square)
    {
        this->acc_type = acc_type;
        switch (trj_mode)
        {
        // case 1:
        //     switch (acc_type)
        //     {
        //     case start:
        //         ad.reset(params->run1.j_max, params->run1.a_max, params->run1.v_max, 0, velocity->v1, FORWARD_LENGTH_START, 0, 0);
        //         break;
        //     case start_half:
        //         ad.reset(params->run1.j_max, params->run1.a_max, params->run1.v_max, 0, velocity->v1, FORWARD_LENGTH_HALF, 0, 0);
        //         break;
        //     case stop:
        //         ad.reset(params->run1.j_max, params->run1.a_max, params->run1.v_max, cur_vel, 0, FORWARD_LENGTH_HALF, 0, 0);
        //         break;
        //     case forward:
        //         ad.reset(params->run1.j_max, params->run1.a_max, params->run1.v_max, cur_vel, velocity->v1, FORWARD_LENGTH * static_cast<float>(num_square), 0, 0);
        //         break;
        //     default:
        //         break;
        //     }
        //     break;
        case 2:
            switch (acc_type)
            {
            case start:
                ad.reset(params->run2.j_max, params->run2.a_max, params->run2.v_max, 0, velocity->v2, FORWARD_LENGTH_START, 0, 0);
                break;
            case start_half:
                ad.reset(params->run2.j_max, params->run2.a_max, params->run2.v_max, 0, velocity->v2, FORWARD_LENGTH_HALF, 0, 0);
                break;
            case stop:
                ad.reset(params->run2.j_max, params->run2.a_max, params->run2.v_max, cur_vel, 0, FORWARD_LENGTH_HALF, 0, 0);
                break;
            case forward:
                ad.reset(params->run2.j_max, params->run2.a_max, params->run2.v_max, cur_vel, velocity->v2, FORWARD_LENGTH * static_cast<float>(num_square), 0, 0);
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        // t_end = ad.t_end();
    }

    void Acceleration::UpdateRef()
    {
        // ref_acc = ad.a(t);
        // ref_vel = ad.v(t);
        // ref_pos = ad.x(t);

        switch (acc_type)
        {
        case start:
            t_end = ad_start.t_end();
            ref_acc = ad_start.a(t);
            ref_vel = ad_start.v(t);
            ref_pos = ad_start.x(t);
            break;
        case start_half:
            t_end = ad_start_half.t_end();
            ref_acc = ad_start_half.a(t);
            ref_vel = ad_start_half.v(t);
            ref_pos = ad_start_half.x(t);
            break;
        case stop:
            t_end = ad_stop.t_end();
            ref_acc = ad_stop.a(t);
            ref_vel = ad_stop.v(t);
            ref_pos = ad_stop.x(t);
            break;
        // case forward:
        //     ad.reset(params->run1.j_max, params->run1.a_max, params->run1.v_max, velocity->v1, velocity->v1, FORWARD_LENGTH * static_cast<float>(num_square), 0, 0);
        //     break;
        default:
            break;
        }

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
            flag_trj = true;
    }

    void Acceleration::Reset()
    {
        flag_read_side_wall = false;
        flag_time = false;
        flag_trj = false;
        t = 0;
    }

    // OfflineTrajectoryBase
    void OfflineTrajectoryBase::Reset()
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