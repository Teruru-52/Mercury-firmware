#include "controller/controller.h"

namespace undercarriage
{
    Controller::Controller(Speaker *speaker,
                           undercarriage::Odometory *odom,
                           PID *pid_angle,
                           PID *pid_rotational_vel,
                           PID *pid_traslational_vel,
                           PID *pid_ir_sensor_front_left,
                           PID *pid_ir_sensor_front_right,
                           PID *pid_ir_sensor_side,
                           undercarriage::Kanayama *kanayama,
                           undercarriage::Dynamic_Feedback *dynamic_feedback,
                           trajectory::Slalom *slalom,
                           trajectory::Acceleration *acc,
                           hardware::IR_Base *ir_base,
                           hardware::IR_Base *ir_is_wall,
                           trajectory::Velocity *velocity)
        : speaker(speaker),
          odom(odom),
          pid_angle(pid_angle),
          pid_rotational_vel(pid_rotational_vel),
          pid_traslational_vel(pid_traslational_vel),
          pid_ir_sensor_front_left(pid_ir_sensor_front_left),
          pid_ir_sensor_front_right(pid_ir_sensor_front_right),
          pid_ir_sensor_side(pid_ir_sensor_side),
          kanayama(kanayama),
          dynamic_feedback(dynamic_feedback),
          slalom(slalom),
          acc(acc),
          mode_ctrl(stop),
          ir_base(ir_base),
          ir_is_wall(ir_is_wall),
          velocity(velocity)
    {
        // ref_size = slalom->GetRefSize();
        // ref_size = acc->GetRefSize();
        // ref_size = pivot_turn180.GetRefSize();
        // ref_size = pivot_turn90.GetRefSize();
        ref_size = 1000;

        log_x = new float[ref_size];
        log_y = new float[ref_size];
        log_theta = new float[ref_size];
        log_l = new float[ref_size];
        log_v = new float[ref_size];
        log_a = new float[ref_size];
        log_ref_l = new float[ref_size];
        log_ref_v = new float[ref_size];
        log_ref_a = new float[ref_size];
        log_omega = new float[ref_size];
        log_kanayama_v = new float[ref_size];
        log_kanayama_w = new float[ref_size];
    }

    void Controller::UpdateOdometory()
    {
        odom->Update();
        cur_pos = odom->GetPosition();
        cur_pos.th += theta_base;
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
            // speaker->ToggleSpeaker();
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
            ref_v = velocity->v2;
            break;
        default:
            break;
        }
    }

    bool Controller::ErrorFlag()
    {
        if (acc_x < -acc_x_err || fabs(cur_vel.x) > vel_x_err)
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
        while (1)
        {
            if (flag_ctrl)
            {
                if (angle == 90)
                    theta_base += M_PI / 2;
                else if (angle == -90)
                    theta_base -= M_PI / 2;
                else if (angle == 180)
                    theta_base += M_PI;
                printf("theta_base: %.1f\n", theta_base);
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Turn(int angle)
    {
        slalom->ResetTrajectory(angle);
        slalom->SetInitialTime(flag_wall_front);
        mode_ctrl = turn;
        while (1)
        {
            if (flag_ctrl)
            {
                if (angle == 90)
                    theta_base += M_PI / 2;
                else if (angle == -90)
                    theta_base -= M_PI / 2;
                printf("theta_base: %.1f\n", theta_base);
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Acceleration(const AccType &acc_type)
    {
        acc->ResetAccCurve(acc_type);
        mode_ctrl = acc_curve;
        while (1)
        {
            if (flag_ctrl)
            {
                theta_base = cur_pos.th;
                printf("theta_base: %.1f\n", theta_base);
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
                theta_base = cur_pos.th;
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
                printf("theta_base: %.1f\n", theta_base);
                ResetCtrl();
                break;
            }
        }
    }

    void Controller::Wait()
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
        u_w = iden_m.GetRotInput(cur_vel);
        InputVelocity(0, u_w);

        if (iden_m.GetFlag())
        {
            Brake();
            flag_ctrl = true;
        }
    }

    void Controller::Step_Iden()
    {
        u_v = iden_step.GetTransInput(cur_vel);
        InputVelocity(u_v, 0);
        if (iden_step.GetFlag())
        {
            Brake();
            flag_ctrl = true;
        }
    }

    void Controller::PartyTrick()
    {
        // u_v = pid_traslational_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_angle->Update(-cur_pos.th) + pid_rotational_vel->Update(-cur_vel.th);
        InputVelocity(u_v, u_w);
    }

    void Controller::SideWallCorrection()
    {
        if (flag_wall_sl)
        {
            if (ir_value.fl > ir_is_wall->fl)
                error_fl = ir_base->fl - static_cast<float>(ir_value.fl);
            else
                error_fl = 0.0;
            if (ir_value.fr < ir_is_wall->fr)
                error_fl *= 1.8;
        }
        else
            error_fl = 0;

        if (flag_wall_sr)
        {
            if (ir_value.fr > ir_is_wall->fr)
                error_fr = ir_base->fr - static_cast<float>(ir_value.fr);
            else
                error_fr = 0.0;
            if (ir_value.fl < ir_is_wall->fl)
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
        }
        else if (mode_ctrl == pivot_turn_left_90)
        {
            pivot_turn90.UpdateRef();
            ref_w = pivot_turn90.GetRefVelocity();
        }
        else if (mode_ctrl == pivot_turn_180)
        {
            pivot_turn180.UpdateRef();
            ref_w = pivot_turn180.GetRefVelocity();
        }
        // u_v = pid_traslational_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_rotational_vel->Update(ref_w - cur_vel.th) + ref_w / Kp_w;
        InputVelocity(u_v, u_w);
        // Logger();
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
        ref_pos.th += theta_base;
        ref_vel = slalom->GetRefVelocity();
        ref_acc = slalom->GetRefAcceleration();
        // Logger();

        kanayama->UpdateRef(ref_pos, ref_vel);
        ref_vel = kanayama->CalcInput(cur_pos);
        u_v = pid_traslational_vel->Update(ref_vel.x - cur_vel.x);
        // u_v = pid_traslational_vel->Update(ref_vel.x - cur_vel.x) + ref_vel.x / Kp_v;
        u_w = pid_rotational_vel->Update(ref_vel.th - cur_vel.th) + ref_vel.th / Kp_w;

        // dynamic_feedback->UpdateRef(ref_pos, ref_vel, ref_acc);
        // ref_vel = dynamic_feedback->CalcInput(cur_pos, cur_vel);
        // u_v = pid_traslational_vel->Update(ref_vel.x - cur_vel.x) + ref_vel.x / Kp_v;
        // u_w = pid_rotational_vel->Update(ref_vel.th - cur_vel.th) + ref_vel.th / Kp_w;
    }

    void Controller::Turn()
    {
        // if (flag_wall_front)
        // {
        //     if (!flag_slalom)
        //     {
        //         if (ir_value.sl > ir_base->sl_slalom || ir_value.sr > ir_base->sr_slalom)
        //         {
        //             flag_slalom = true;
        //             // cur_pos.x = 0.01;
        //             // cur_pos.y = 0.0;
        //             // odom->OverWritePos(cur_pos);
        //         }
        //         else
        //         {
        //             SideWallCorrection();
        //             u_v = pid_traslational_vel->Update(ref_v - cur_vel.x) + ref_v / Kp_v;
        //             // u_w = pid_ir_sensor_side->Update(error_fl - error_fr) + pid_angle->Update(theta_base - cur_pos.th);
        //             u_w = pid_angle->Update(theta_base - cur_pos.th);
        //         }
        //     }
        //     else
        //         CalcSlalomInput();
        // }
        // else
        CalcSlalomInput();

        InputVelocity(u_v, u_w);
        // Logger();
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
        // kanayama->UpdateRef(ref_pos, ref_vel);
        // ref_vel = kanayama->CalcInput(cur_pos);
        // Logger();
        u_v = pid_traslational_vel->Update(ref_vel.x - cur_vel.x) + (Tp1_v * ref_acc.x + ref_vel.x) / Kp_v;
        // u_v = pid_traslational_vel->Update(ref_vel.x - cur_vel.x);

        if (flag_side_correct)
            u_w = pid_ir_sensor_side->Update(error_fl - error_fr) + pid_angle->Update(theta_base - cur_pos.th);
        else
            u_w = pid_angle->Update(theta_base - cur_pos.th);

        InputVelocity(u_v, u_w);
        if (acc->Finished())
            flag_ctrl = true;
    }

    void Controller::GoStraight(float ref_l)
    {
        if ((length > ref_l * WALL_TIMING) && (!flag_straight_time))
        {
            flag_straight_time = true;
            flag_straight_wall = true;
        }
        if (length < ref_l)
        {
            SideWallCorrection();
            u_v = pid_traslational_vel->Update(ref_v - cur_vel.x) + ref_v / Kp_v;
            if (flag_side_correct)
                u_w = pid_ir_sensor_side->Update(error_fl - error_fr) + pid_angle->Update(theta_base - cur_pos.th);
            else
                u_w = pid_angle->Update(theta_base - cur_pos.th);
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
            flag_ctrl = true;
        }
    }

    void Controller::Wait(int time)
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
            float error_sl = ir_base->sl - static_cast<float>(ir_value.sl);
            float error_sr = ir_base->sr - static_cast<float>(ir_value.sr);
            v_left = pid_ir_sensor_front_left->Update(error_sl);
            v_right = pid_ir_sensor_front_right->Update(error_sr);
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

        // if (cnt_blind_alley % 1 == 0)
        // {
        //     flag_maze_load = true;
        // }
        // else
        // {
        PivotTurn(-90);
        FrontWallCorrection();
        PivotTurn(-90);
        Back();
        Acceleration(AccType::start);
        // }
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
        kanayama->Reset();
        dynamic_feedback->Reset();
        pivot_turn180.Reset();
        pivot_turn90.Reset();
        slalom->Reset();
        acc->Reset();

        pid_angle->ResetPID();
        pid_rotational_vel->ResetPID();
        pid_traslational_vel->ResetPID();
        pid_ir_sensor_front_left->ResetPID();
        pid_ir_sensor_front_right->ResetPID();
        pid_ir_sensor_side->ResetPID();

        odom->Reset();
        odom->ResetTheta();
        // theta_base = 0.0;
        mode_ctrl = stop;

        flag_slalom = false;
        flag_side_correct = false;
        index_log = 0;
        cnt_time = 0;
        flag_ctrl = false;
    }

    Direction Controller::getWallData()
    {
        Toggle_GPIO(BACK_LEFT_LED);
        Direction wall;

        int8_t robot_dir_index = 0;
        while (1)
        {
            if (robot_dir.byte == NORTH << robot_dir_index)
                break;
            robot_dir_index++;
        }

        if (ir_wall_value.sl > ir_is_wall->sl || ir_wall_value.sr > ir_is_wall->sr)
        {
            wall.byte |= NORTH << robot_dir_index;
            flag_wall_front = true;
        }
        else
            flag_wall_front = false;

        if (ir_wall_value.fl > ir_is_wall->fl)
        {
            wall.byte |= NORTH << (robot_dir_index + 3) % 4;
            flag_wall_sl = true;
        }
        else
            flag_wall_sl = false;

        if (ir_wall_value.fr > ir_is_wall->fr)
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
            Wait(wait_time);
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
                if (ir_value.sl > ir_is_wall->sl && ir_value.sr > ir_is_wall->sr)
                    FrontWallCorrection();
                PivotTurn(-90);
                if (prev_wall_cnt == 2)
                    cnt_can_back++;
                printf("cnt: %d\n", cnt_can_back);
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
                if (ir_value.sl > ir_is_wall->sl && ir_value.sr > ir_is_wall->sr)
                    FrontWallCorrection();
                PivotTurn(90);
                if (prev_wall_cnt == 2)
                    cnt_can_back++;
                printf("cnt: %d\n", cnt_can_back);
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
            // if (ENABLE_SLALOM)
            GoStraight();
            // else
            //     Acceleration(AccType::forward0);
            break;

        case Operation::TURN_LEFT90:
            if (ENABLE_SLALOM)
                Turn(90);
            else
            {
                if (ir_value.sl > ir_is_wall->sl && ir_value.sr > ir_is_wall->sr)
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
                if (ir_value.sl > ir_is_wall->sl && ir_value.sr > ir_is_wall->sr)
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
        log_theta[index_log] = cur_pos.th;
        log_l[index_log] = length;
        log_v[index_log] = cur_vel.x;
        log_a[index_log] = acc_x;
        log_ref_l[index_log] = ref_pos.x;
        log_ref_v[index_log] = ref_vel.x;
        log_ref_a[index_log] = ref_acc.x * cos(cur_pos.th);
        log_omega[index_log] = cur_vel.th;
        if (mode_ctrl == turn)
        {
            log_kanayama_v[index_log] = ref_vel.x;
            log_kanayama_w[index_log] = ref_vel.th;
        }
        log_x[index_log] = ref_pos.x;
        log_y[index_log] = ref_pos.y;
        log_theta[index_log] = ref_pos.th;
        log_omega[index_log] = ref_vel.th;
        index_log++;
    }

    void Controller::OutputLog()
    {
        odom->OutputLog();
        // printf("%f, %f\n", u_v, u_w);
    }

    void Controller::OutputSlalomLog()
    {
        for (int i = 0; i < ref_size; i++)
        {
            // printf("%f, %f, %f, %f, %f\n", log_x[i], log_y[i], log_theta[i], log_kanayama_v[i], log_kanayama_w[i]);
            // printf("%f, %f, %f, %f, %f, %f, %f\n", log_x[i], log_y[i], log_theta[i], log_v[i], log_omega[i], log_kanayama_v[i], log_kanayama_w[i]);
            printf("%f, %f, %f, %f\n", log_x[i], log_y[i], log_theta[i], log_omega[i]);
        }
    }

    void Controller::OutputPivotTurnLog()
    {
        for (int i = 0; i < ref_size; i++)
            printf("%f, %f\n", log_theta[i], log_omega[i]);
    }

    void Controller::OutputTranslationLog()
    {
        for (int i = 0; i < ref_size; i++)
            printf("%f, %f, %f, %f, %f, %f\n", log_l[i], log_v[i], log_a[i], log_ref_l[i], log_ref_v[i], log_ref_a[i]);
    }

    void Controller::MotorTest(float v_left, float v_right)
    {
        motor.Drive(v_left, v_right); // voltage [V]
    }
}