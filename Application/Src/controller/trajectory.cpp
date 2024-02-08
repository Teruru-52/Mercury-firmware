/**
 * @file trajectory.cpp
 * @brief trajectory generation for undercarriage
 * @author Teruru-52
 */

#include "controller/trajectory.h"

namespace trajectory
{
    void OnlineTrajectoryBase::Reset()
    {
        flag_time = false;
        flag_trj = false;
        t = 0;
    }

    // Slalom
    void Slalom::ResetTrajectory(const SlalomType &slalom_type, float ref_theta, ctrl::Pose cur_pos)
    {
        this->slalom_type = slalom_type;
        state.q.x = state.q.y = 0;
        switch (trj_mode)
        {
        case 1:
            v_ref = velocity->v1;
            param = params->run1;
            break;
        case 2:
            v_ref = velocity->v2;
            param = params->run2;
            break;
        case 3:
            v_ref = velocity->v3;
            param = params->run3;
            break;
        case 4:
            v_ref = velocity->v4;
            param = params->run4;
            break;
        case 5:
            v_ref = velocity->v5;
            param = params->run5;
            break;
        default:
            break;
        }
        switch (slalom_type)
        {
        case left_90:
            ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, 90, ref_theta), 80, 0, param.j_max, param.a_max, param.v_max);
            break;
        case right_90:
            ss = ctrl::slalom::Shape(ctrl::Pose(90 - cur_pos.x, -90, ref_theta), -80, 0, param.j_max, param.a_max, param.v_max);
            break;
        case left_45:
            ss = ctrl::slalom::Shape(ctrl::Pose(90, 90, M_PI * 0.25f), 0, 0, param.j_max, param.a_max, param.v_max);
            break;
        case right_45:
            ss = ctrl::slalom::Shape(ctrl::Pose(90, -90, -M_PI * 0.25f), 0, 0, param.j_max, param.a_max, param.v_max);
            break;
        default:
            break;
        }

        st = ctrl::slalom::Trajectory(ss);
        st.reset(v_ref, 0, 0);
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

    // Acceleration
    void Acceleration::ResetTrajectory(const AccType &acc_type, float cur_vel, uint8_t num_square)
    {
        this->acc_type = acc_type;
        switch (trj_mode)
        {
        case 1:
            v_ref = velocity->v1;
            param = params->run1;
            break;
        case 2:
            v_ref = velocity->v2;
            param = params->run2;
            break;
        case 3:
            v_ref = velocity->v3;
            param = params->run3;
            break;
        case 4:
            v_ref = velocity->v4;
            param = params->run4;
            break;
        case 5:
            v_ref = velocity->v5;
            param = params->run5;
            break;
        default:
            break;
        }
        switch (acc_type)
        {
        case start:
            ad.reset(param.j_max, param.a_max, param.v_max, 0, v_ref, FORWARD_LENGTH_START, 0, 0);
            break;
        case start_half:
            ad.reset(param.j_max, param.a_max, param.v_max, 0, v_ref, FORWARD_LENGTH_HALF, 0, 0);
            break;
        case stop:
            ad.reset(param.j_max, param.a_max, param.v_max, cur_vel, 0, FORWARD_LENGTH_HALF, 0, 0);
            break;
        case forward:
            ad.reset(param.j_max, param.a_max, param.v_max, cur_vel, v_ref, FORWARD_LENGTH * static_cast<float>(num_square), 0, 0);
            break;
        default:
            break;
        }
        t_end = ad.t_end();
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
            flag_trj = true;
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