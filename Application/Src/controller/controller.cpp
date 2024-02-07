/**
 * @file controller.cpp
 * @author Teruru-52
 */

#include "controller/controller.h"

namespace undercarriage
{
    Controller::Controller(undercarriage::Odometory *odom,
                           PID_Instances *pid,
                           undercarriage::TrackerBase *tracker,
                           trajectory::Slalom *slalom,
                           trajectory::Acceleration *acc,
                           hardware::IR_Param *ir_param,
                           trajectory::Velocity *velocity)
        : odom(odom),
          pid(pid),
          tracker(tracker),
          slalom(slalom),
          acc(acc),
          mode_ctrl(stop),
          ir_param(ir_param),
          velocity(velocity)
    {
        if (ENABLE_LOG)
        {
            // ref_size = slalom->GetRefSize();
            // ref_size = acc->GetRefSize();
            // ref_size = pivot_turn180.GetRefSize();
            // ref_size = pivot_turn90.GetRefSize();
            ref_size = 900;

            log_x = new float[ref_size];
            log_y = new float[ref_size];
            // log_l = new float[ref_size];
            log_theta = new float[ref_size];
            log_omega = new float[ref_size];
            log_v = new float[ref_size];
            log_a = new float[ref_size];
            log_ref_x = new float[ref_size];
            log_ref_y = new float[ref_size];
            log_ref_theta = new float[ref_size];
            log_ref_omega = new float[ref_size];
            log_ref_v = new float[ref_size];
            log_ref_a = new float[ref_size];
            log_ctrl_v = new float[ref_size];
            log_ctrl_w = new float[ref_size];
            log_u_v = new float[ref_size];
            log_u_w = new float[ref_size];
        }
    }

    void Controller::UpdateOdometory()
    {
        odom->Update();
        cur_pos = odom->GetPosition();
        theta_global = theta_base - theta_error + cur_pos.th;
        cur_vel = odom->GetVelocity();
        length = odom->GetLength();
        acc_x = odom->GetAccX();
    }

    void Controller::SetIRdata(const IR_Value &ir_value)
    {
        this->ir_value = ir_value;
        if (slalom->GetWallFlag() || acc->GetWallFlag() || flag_straight_wall)
        {
            updateWallData();
            slalom->ResetWallFlag();
            acc->ResetWallFlag();
            flag_straight_wall = false;
            flag_wall = true;
            // speaker.ToggleSpeaker();
        }
    }

    void Controller::SetTrajectoryMode(int trj_mode)
    {
        slalom->SetMode(trj_mode);
        acc->SetMode(trj_mode);

        switch (trj_mode)
        {
        case 1:
            ref_v = velocity->v1;
            break;
        case 2:
            ref_v = velocity->v2;
            break;
        case 3:
            ref_v = velocity->v3;
            break;
        default:
            break;
        }
    }

    bool Controller::ErrorFlag()
    {
        if (acc_x < -acc_x_err)
            flag_safety = true;
        return flag_safety;
    }

    void Controller::SetM_Iden()
    {
        mode_ctrl = m_iden;
        while (1)
        {
            if (flag_ctrl)
            {
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::SetStep_Iden()
    {
        mode_ctrl = step_iden;
        while (1)
        {
            if (flag_ctrl)
            {
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::SetPartyTrick()
    {
        mode_ctrl = party_trick;
        while (1)
        {
        }
    }

    void Controller::PivotTurn(int angle)
    {
        if (angle == 90)
            mode_ctrl = pivot_turn_left_90;
        else if (angle == -90)
            mode_ctrl = pivot_turn_right_90;
        else if (angle == 180)
            mode_ctrl = pivot_turn_180;
        ref_theta = static_cast<float>(angle) * M_PI / 180.0;

        while (1)
        {
            if (flag_ctrl)
            {
                theta_base += ref_theta;
                theta_error += (ref_theta - cur_pos.th);
                // printf("theta_base: %.1f\n", theta_base);
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Turn(int angle)
    {
        angle_turn = angle;
        ref_theta = static_cast<float>(angle) * M_PI / 180.0f;
        slalom->ResetTrajectory(angle, ref_theta - theta_error, cur_pos);
        tracker->SetXi(cur_vel.x);
        mode_ctrl = turn;

        while (1)
        {
            if (flag_ctrl)
            {
                theta_base += ref_theta;
                theta_error += (ref_theta - cur_pos.th);
                // printf("theta_base: %.1f\n", theta_base);
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Acceleration(const AccType &acc_type, uint8_t num_square)
    {
        acc->ResetTrajectory(acc_type, cur_vel.x, num_square);
        mode_ctrl = acc_curve;
        while (1)
        {
            if (flag_ctrl)
            {
                theta_error += -cur_pos.th;
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::GoStraight()
    {
        mode_ctrl = forward;
        while (1)
        {
            if (flag_ctrl)
            {
                theta_error += -cur_pos.th;
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::FrontWallCorrection()
    {
        mode_ctrl = front_wall_correction;
        while (1)
        {
            if (flag_ctrl)
            {
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Back()
    {
        mode_ctrl = back;
        while (1)
        {
            if (flag_ctrl)
            {
                odom->ResetTheta();
                theta_base = 0.0;
                // printf("theta_base: %.1f\n", theta_base);
                theta_error = 0.0;
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Wait_ms()
    {
        mode_ctrl = wait;
        while (1)
        {
            if (flag_ctrl)
            {
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::M_Iden()
    {
        u_w = iden_m.GetInput(cur_vel.th);
        InputVelocity(0, u_w);

        // u_v = iden_m.GetInput(cur_vel.x);
        // InputVelocity(u_v, 0);

        if (iden_m.Finished())
        {
            Brake();
            flag_ctrl = true;
        }
    }

    void Controller::Step_Iden()
    {
        // u_w = iden_step.GetInput(cur_vel.th);
        // InputVelocity(0, u_w);

        u_v = iden_step.GetInput(cur_vel.x);
        InputVelocity(u_v, 0);

        if (iden_step.Finished())
        {
            Brake();
            flag_ctrl = true;
        }
    }

    void Controller::PartyTrick()
    {
        // u_v = pid->trans_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid->angle->Update(-cur_pos.th) + pid->rot_vel->Update(-cur_vel.th);
        InputVelocity(u_v, u_w);
    }

    void Controller::SideWallCorrection()
    {
        if (flag_wall_sl)
        {
            if (ir_value.fl > ir_param->is_wall.fl)
                error_fl = ir_param->ctrl_base.fl - static_cast<float>(ir_value.fl);
            else
                error_fl = 0.0;
            if (ir_value.fr < ir_param->is_wall.fr)
                error_fl *= 1.8;
        }
        else
            error_fl = 0;

        if (flag_wall_sr)
        {
            if (ir_value.fr > ir_param->is_wall.fr)
                error_fr = ir_param->ctrl_base.fr - static_cast<float>(ir_value.fr);
            else
                error_fr = 0.0;
            if (ir_value.fl < ir_param->is_wall.fl)
                error_fr *= 1.8;
        }
        else
            error_fr = 0;
    }

    void Controller::PivotTurn()
    {
        if (mode_ctrl == pivot_turn_right_90)
        {
            pivot_turn90.UpdateRef();
            ref_w = -pivot_turn90.GetRefVelocity();
            ref_dw = -pivot_turn90.GetRefAcceleration();
        }
        else if (mode_ctrl == pivot_turn_left_90)
        {
            pivot_turn90.UpdateRef();
            ref_w = pivot_turn90.GetRefVelocity();
            ref_dw = pivot_turn90.GetRefAcceleration();
        }
        else if (mode_ctrl == pivot_turn_180)
        {
            pivot_turn180.UpdateRef();
            ref_w = pivot_turn180.GetRefVelocity();
        }
        // u_v = pid->trans_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid->rot_vel->Update(ref_w - cur_vel.th) + ref_w / Kp_w;
        // u_w = pid->rot_vel->Update(ref_w - cur_vel.th) + (Tp1_w * ref_dw + ref_w) / Kp_w;
        InputVelocity(u_v, u_w);
        if (ENABLE_LOG)
            Logger();
        if (pivot_turn90.Finished() || pivot_turn180.Finished())
        {
            Brake();
            flag_ctrl = true;
        }
    }

    void Controller::CalcSlalomInput()
    {
        slalom->UpdateRef();
        ref_pos = slalom->GetRefPosition();
        ref_vel = slalom->GetRefVelocity();
        ref_acc = slalom->GetRefAcceleration();

        tracker->UpdateRef(ref_pos, ref_vel, ref_acc);
        ref_vel_ctrl = tracker->CalcInput(cur_pos, cur_vel);
        // u_v = pid->trans_vel->Update(ref_vel_ctrl.x - cur_vel.x);
        u_v = pid->trans_vel->Update(ref_vel_ctrl.x - cur_vel.x) + ref_vel_ctrl.x / Kp_v;
        u_w = pid->rot_vel->Update(ref_vel_ctrl.th - cur_vel.th) + ref_vel_ctrl.th / Kp_w;
    }

    float Controller::GetFrontWallPos(float ir_fmean)
    {
        if (ir_param->log.b * ir_fmean + ir_param->log.c > 0)
            return ir_param->log.a * log(ir_param->log.b * ir_fmean + ir_param->log.c) + ir_param->log.d;
        else
            return ir_param->log.a * log(1e-10) + ir_param->log.d;
    };

    void Controller::Turn()
    {
        // if (flag_wall_front) // 前壁があれば前壁補正
        // {
        //     if (!flag_slalom) // 前壁があり，slalomが始まっていなければ前壁補正
        //     {
        //         float ir_fmean = (ir_value.sl + ir_value.sr) * 0.5;
        //         if (ir_fmean > ir_param->ctrl_base.slalom) // しきい値より前壁センサの平均値が大きければ，位置を上書きしてslalom開始
        //         {
        //             flag_slalom = true;
        //             cur_pos.x = GetFrontWallPos(ir_fmean);
        //             cur_pos.y = 0.0;
        //             odom->OverWritePos(cur_pos);
        //             slalom->ResetTrajectory(angle_turn, ref_theta - theta_error, cur_pos);
        //             CalcSlalomInput();
        //         }
        //         else // しきい値より前壁センサの平均値が小さければ，slalomを開始せずに直進する
        //         {
        //             SideWallCorrection();
        //             u_v = pid->trans_vel->Update(ref_v - cur_vel.x) + ref_v / Kp_v;
        //             // u_w = pid->ir_side->Update(error_fl - error_fr) + pid->angle->Update(theta_base - theta_global);
        //             u_w = pid->angle->Update(theta_base - theta_global);
        //         }
        //     }
        //     else // 前壁があり，slalomが始まり次第この処理をする
        //         CalcSlalomInput();
        // }
        // else // 前壁がない場合
        CalcSlalomInput();

        InputVelocity(u_v, u_w);
        if (ENABLE_LOG)
            Logger();
        if (slalom->Finished())
            flag_ctrl = true;
    }

    void Controller::Acceleration()
    {
        SideWallCorrection();
        acc->UpdateRef();
        ref_pos.x = acc->GetRefPosition();
        ref_pos.y = 0.0;
        ref_pos.th = theta_base;
        ref_vel.x = acc->GetRefVelocity();
        ref_vel.y = 0.0;
        ref_acc.x = acc->GetRefAcceleration();
        ref_acc.y = 0.0;
        if (ENABLE_LOG)
            Logger();
        u_v = pid->trans_vel->Update(ref_vel.x - cur_vel.x) + (Tp1_v * ref_acc.x + ref_vel.x) / Kp_v;
        // u_v = pid->trans_vel->Update(ref_vel.x - cur_vel.x);

        if (flag_side_correct)
            u_w = pid->ir_side->Update(error_fl - error_fr) + pid->angle->Update(theta_base - theta_global);
        else
            u_w = pid->angle->Update(theta_base - theta_global);

        InputVelocity(u_v, u_w);
        if (acc->Finished())
            flag_ctrl = true;
    }

    void Controller::GoStraight(float ref_l)
    {
        if ((cur_pos.x > ref_l * WALL_TIMING) && (!flag_straight_time))
        {
            flag_straight_time = true;
            flag_straight_wall = true;
        }
        if (cur_pos.x < ref_l)
        {
            SideWallCorrection();
            u_v = pid->trans_vel->Update(ref_v - cur_vel.x) + ref_v / Kp_v;
            if (flag_side_correct)
                u_w = pid->ir_side->Update(error_fl - error_fr) + pid->angle->Update(theta_base - theta_global);
            else
                u_w = pid->angle->Update(theta_base - theta_global);
            InputVelocity(u_v, u_w);
        }
        else
        {
            flag_ctrl = true;
            flag_straight_time = false;
        }
    }

    void Controller::Back(int time)
    {
        if (cnt_time < time)
        {
            InputVelocity(-0.5, 0.0);
            cnt_time++;
        }
        else
        {
            Brake();
            odom->ResetEncoder();
            flag_maze_load = true;
            flag_ctrl = true;
        }
    }

    void Controller::Wait_ms(int time)
    {
        if (cnt_time < time)
        {
            motor.Brake();
            cnt_time++;
        }
        else
            flag_ctrl = true;
    }

    void Controller::FrontWallCorrection(const IR_Value &ir_value)
    {
        if (cnt_time < correction_time)
        {
            float error_sl = ir_param->ctrl_base.sl - static_cast<float>(ir_value.sl);
            float error_sr = ir_param->ctrl_base.sr - static_cast<float>(ir_value.sr);
            v_left = pid->ir_front_l->Update(error_sl);
            v_right = pid->ir_front_r->Update(error_sr);
            motor.Drive(v_left, v_right);
            cnt_time++;
        }
        else
            flag_ctrl = true;
    }

    void Controller::BlindAlley()
    {
        Acceleration(AccType::stop);
        FrontWallCorrection();
        PivotTurn(-90);
        FrontWallCorrection();
        PivotTurn(-90);
        Back();
        Acceleration(AccType::start);
        cnt_can_back = 0;
    }

    void Controller::StartMove()
    {
        PivotTurn(-90);
        FrontWallCorrection();
        PivotTurn(90);
        Back();
        Acceleration(AccType::start);
    }

    void Controller::InitializePosition()
    {
        PivotTurn(90);
        FrontWallCorrection();
        PivotTurn(90);
        Back();
    }

    void Controller::Brake()
    {
        mode_ctrl = stop;
        motor.Brake();
    }

    void Controller::InputVelocity(float input_v, float input_w)
    {
        v_left = input_v - input_w;
        v_right = input_v + input_w;
        motor.Drive(v_left, v_right);
    }

    void Controller::ResetCtrl()
    {
        tracker->Reset();
        pivot_turn180.Reset();
        pivot_turn90.Reset();
        slalom->Reset();
        acc->Reset();
        pid->Reset();

        odom->Reset();
        odom->ResetTheta();
        // theta_base = 0.0;
        // mode_ctrl = stop;

        flag_slalom = false;
        flag_side_correct = false;
        index_log = 0;
        cnt_time = 0;
        flag_ctrl = false;
    }

    Direction Controller::getWallData()
    {
        Direction wall;

        int8_t robot_dir_index = 0;
        while (1)
        {
            if (robot_dir.byte == NORTH << robot_dir_index)
                break;
            robot_dir_index++;
        }

        if (ir_wall_value.sl > ir_param->is_wall.sl || ir_wall_value.sr > ir_param->is_wall.sr)
        {
            wall.byte |= NORTH << robot_dir_index;
            flag_wall_front = true;
        }
        else
            flag_wall_front = false;

        if (ir_wall_value.fl > ir_param->is_wall.fl)
        {
            wall.byte |= NORTH << (robot_dir_index + 3) % 4;
            flag_wall_sl = true;
        }
        else
            flag_wall_sl = false;

        if (ir_wall_value.fr > ir_param->is_wall.fr)
        {
            wall.byte |= NORTH << (robot_dir_index + 1) % 4;
            flag_wall_sr = true;
        }
        else
            flag_wall_sr = false;

        prev_wall_cnt = wall.nWall();

        return wall;
    }

    void Controller::UpdatePos(const Direction &dir)
    {
        if (NORTH == dir.byte)
            robot_position += IndexVec::vecNorth;
        else if (SOUTH == dir.byte)
            robot_position += IndexVec::vecSouth;
        else if (EAST == dir.byte)
            robot_position += IndexVec::vecEast;
        else if (WEST == dir.byte)
            robot_position += IndexVec::vecWest;
    }

    void Controller::robotMove()
    {
        switch (mode_ctrl)
        {
        case forward:
            GoStraight(FORWARD_LENGTH);
            break;
        case acc_curve:
            Acceleration();
            break;
        case turn:
            Turn();
            break;
        case pivot_turn_right_90:
            PivotTurn();
            break;
        case pivot_turn_left_90:
            PivotTurn();
            break;
        case pivot_turn_180:
            PivotTurn();
            break;
        case front_wall_correction:
            FrontWallCorrection(ir_value);
            break;
        case back:
            Back(back_time);
            break;
        case m_iden:
            M_Iden();
            break;
        case step_iden:
            Step_Iden();
            break;
        case party_trick:
            PartyTrick();
            break;
        case stop:
            Brake();
            break;
        case wait:
            Wait_ms(wait_time);
            break;
        default:
            break;
        }
    }

    void Controller::DirMove(const Direction &dir)
    {
        int8_t robot_dir_index = 0;
        while (1)
        {
            if (robot_dir.byte == NORTH << robot_dir_index)
                break;
            robot_dir_index++;
        }

        int8_t next_dir_index = 0;
        while (1)
        {
            if (dir.byte == NORTH << next_dir_index)
                break;
            next_dir_index++;
        }

        dir_diff = next_dir_index - robot_dir_index;
        // printf("dir_diff = %d\n", dir_diff);
        // Straight Line
        if (dir_diff == 0)
        {
            flag_side_correct = true;
            GoStraight();
        }
        // Turn Right
        else if (dir_diff == 1 || dir_diff == -3)
        {
            if (ENABLE_SLALOM)
                Turn(-90);
            else
            {
                Acceleration(AccType::stop);
                if (ir_value.sl > ir_param->is_wall.sl && ir_value.sr > ir_param->is_wall.sr)
                    FrontWallCorrection();
                PivotTurn(-90);
                if (prev_wall_cnt == 2)
                    cnt_can_back++;
                // printf("cnt: %d\n", cnt_can_back);
                if (cnt_can_back >= CNT_BACK && flag_wall_front && flag_wall_sl)
                {
                    Back();
                    Acceleration(AccType::start);
                    cnt_can_back = 0;
                }
                else
                    Acceleration(AccType::start_half);
            }
        }
        // Turn Left
        else if (dir_diff == -1 || dir_diff == 3)
        {
            if (ENABLE_SLALOM)
                Turn(90);
            else
            {
                Acceleration(AccType::stop);
                if (ir_value.sl > ir_param->is_wall.sl && ir_value.sr > ir_param->is_wall.sr)
                    FrontWallCorrection();
                PivotTurn(90);
                if (prev_wall_cnt == 2)
                    cnt_can_back++;
                // printf("cnt: %d\n", cnt_can_back);
                if (cnt_can_back >= CNT_BACK && flag_wall_front && flag_wall_sl)
                {
                    Back();
                    Acceleration(AccType::start);
                    cnt_can_back = 0;
                }
                else
                    Acceleration(AccType::start_half);
            }
        }
        // U-Turn
        else
        {
            if (prev_wall_cnt == 3) // Blind Alley
            {
                cnt_blind_alley++;
                BlindAlley();
            }
            else
            {
                Acceleration(AccType::stop);
                PivotTurn(180);
                Acceleration(AccType::start_half);
            }
        }
    }

    void Controller::OpMove(const Operation &op)
    {
        switch (op.op)
        {
        case Operation::FORWARD:
            flag_side_correct = true;
            for (int i = op.n; i > 0; i--)
                GoStraight();
            // Acceleration(AccType::forward, op.n);
            break;

        case Operation::TURN_LEFT90:
            if (ENABLE_SLALOM)
                Turn(90);
            else
            {
                Acceleration(AccType::stop);
                if (ir_value.sl > ir_param->is_wall.sl && ir_value.sr > ir_param->is_wall.sr)
                    FrontWallCorrection();
                PivotTurn(90);
                Acceleration(AccType::start_half);
            }
            break;

        case Operation::TURN_RIGHT90:
            if (ENABLE_SLALOM)
                Turn(-90);
            else
            {
                Acceleration(AccType::stop);
                if (ir_value.sl > ir_param->is_wall.sl && ir_value.sr > ir_param->is_wall.sr)
                    FrontWallCorrection();
                PivotTurn(-90);
                Acceleration(AccType::start_half);
            }
            break;

        case Operation::STOP:
            Acceleration(AccType::stop);
            Brake();
            break;
        default:
            break;
        }
    }

    void Controller::Logger()
    {
        log_x[index_log] = cur_pos.x;
        log_y[index_log] = cur_pos.y;
        // log_l[index_log] = length;
        log_theta[index_log] = cur_pos.th;
        log_omega[index_log] = cur_vel.th;
        log_v[index_log] = cur_vel.x;
        log_a[index_log] = acc_x;
        log_ref_x[index_log] = ref_pos.x;
        log_ref_y[index_log] = ref_pos.y;
        log_ref_theta[index_log] = ref_pos.th;
        log_ref_omega[index_log] = ref_vel.th;
        log_ref_v[index_log] = ref_vel.x;

        if (mode_ctrl == turn)
        {
            log_ref_a[index_log] = ref_acc.x * cos(ref_pos.th) + ref_acc.y * sin(ref_pos.th);
            log_ctrl_v[index_log] = ref_vel_ctrl.x;
            log_ctrl_w[index_log] = ref_vel_ctrl.th;
        }
        if (mode_ctrl == acc_curve || mode_ctrl == forward)
            log_ref_a[index_log] = ref_acc.x;

        log_u_v[index_log] = u_v;
        log_u_w[index_log] = u_w;
        index_log++;
    }

    void Controller::OutputLog()
    {
        odom->OutputLog();
        // printf("%.2f, %.2f\n", u_v, u_w);
    }

    void Controller::OutputSlalomLog()
    {
        ref_size = slalom->GetRefSize();
        for (int i = 0; i < ref_size; i++)
        {
            printf("%.2f, %.2f, %.3f, %.2f, %.3f, %.1f, ", log_x[i], log_y[i], log_theta[i], log_v[i], log_omega[i], log_a[i]);
            printf("%.2f, %.2f, %.3f, %.1f, %.3f, %.1f, ", log_ref_x[i], log_ref_y[i], log_ref_theta[i], log_ref_v[i], log_ref_omega[i], log_ref_a[i]);
            printf("%.2f, %.3f, %.3f, %.3f\n", log_ctrl_v[i], log_ctrl_w[i], log_u_v[i], log_u_w[i]);
        }
    }

    void Controller::OutputPivotTurnLog()
    {
        if (mode_ctrl == pivot_turn_180)
            ref_size = pivot_turn180.GetRefSize();
        else
            ref_size = pivot_turn90.GetRefSize();
        for (int i = 0; i < ref_size; i++)
            printf("%.2f, %.2f\n", log_theta[i], log_omega[i]);
    }

    void Controller::OutputTranslationLog()
    {
        if (mode_ctrl == acc_curve)
            ref_size = acc->GetRefSize();
        for (int i = 0; i < ref_size; i++)
        {
            printf("%.3f, %.3f, %.3f, %.3f, ", log_x[i], log_y[i], log_v[i], log_a[i]);
            printf("%.3f, %.3f, %.3f, ", log_ref_x[i], log_ref_v[i], log_ref_a[i]);
            printf("%.3f, %.3f\n", log_u_v[i], log_u_w[i]);
        }
    }

    void Controller::MotorTest(float v_left, float v_right)
    {
        motor.Drive(v_left, v_right); // voltage [V]
    }
}