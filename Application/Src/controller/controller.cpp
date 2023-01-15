#include "controller/controller.h"

namespace undercarriage
{
    Controller::Controller(undercarriage::Odometory *odom,
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
                           const std::vector<float> &ir_parameters)
        : odom(odom),
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
          mode(stop),
          ir_wall_base(ir_parameters[0]),
          ir_fl_base(ir_parameters[1]),
          ir_fr_base(ir_parameters[2]),
          ir_sl_base(ir_parameters[3]),
          ir_sr_base(ir_parameters[4]),
          flag_controller(false),
          flag_wall(true),
          flag_safety(false),
          cnt(0),
          index_log(0),
          dir_diff(0),
          robot_position(0, 0),
          robot_dir(NORTH)
    {
        // ref_size = slalom->GetRefSize();
        // ref_size = acc->GetRefSize();
        // ref_size = pivot_turn180.GetRefSize();
        // ref_size = pivot_turn90.GetRefSize();

        // log_x = new float[ref_size];
        // log_y = new float[ref_size];
        // log_theta = new float[ref_size];
        // log_l = new float[ref_size];
        // log_v = new float[ref_size];
        // log_a = new float[ref_size];
        // log_ref_l = new float[ref_size];
        // log_ref_v = new float[ref_size];
        // log_ref_a = new float[ref_size];
        // log_omega = new float[ref_size];
        // log_kanayama_v = new float[ref_size];
        // log_kanayama_w = new float[ref_size];
    }

    void Controller::InitializeOdometory()
    {
        odom->Initialize();
    }

    void Controller::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Controller::UpdateOdometory()
    {
        odom->Update();
        l = odom->GetLength();
        cur_pos = odom->GetPosition();
        cur_vel = odom->GetVelocity();
        acc_x = odom->GetAccX();
    }

    bool Controller::ErrorFlag()
    {
        if (acc_x < -30.0 || fabs(cur_vel[1]) > 20.0)
            flag_safety = true;
        return flag_safety;
    }

    void Controller::ResetOdometory()
    {
        odom->Reset();
    }

    int16_t Controller::GetPulse()
    {
        int16_t pulse = odom->GetPulse();
        return pulse;
    }

    void Controller::UpdateIMU()
    {
        odom->UpdateIMU();
    }

    // void Controller::SetBase()
    // {
    // base_theta = cur_pos[2];
    // }

    void Controller::SetIRdata(const std::vector<uint32_t> &ir_value)
    {
        ir_data = ir_value;
    }

    void Controller::SetTrajectoryMode(int mode)
    {
        slalom->SetMode(mode);
        acc->SetMode(mode);
    }

    void Controller::PivotTurn(int angle)
    {
        if (angle == 90)
        {
            mode = pivot_turn_left_90;
        }
        else if (angle == -90)
        {
            mode = pivot_turn_right_90;
        }
        else if (angle == 180)
        {
            mode = pivot_turn_180;
        }
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::Turn(int angle)
    {
        slalom->ResetTrajectory(angle);
        mode = turn;
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::Acceleration(const AccType &acc_type)
    {
        acc->ResetAccCurve(acc_type);
        mode = acc_curve;
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::GoStraight()
    {
        mode = forward;
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::FrontWallCorrection()
    {
        mode = front_wall_correction;
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::Back()
    {
        mode = back;
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::Wait()
    {
        mode = wait;
        while (1)
        {
            if (flag_controller)
            {
                Reset();
                break;
            }
        }
    }

    void Controller::PartyTrick()
    {
        // u_v = pid_traslational_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_angle->Update(-cur_pos[2]) + pid_rotational_vel->Update(-cur_vel[1]);
        InputVelocity(u_v, u_w);
    }

    void Controller::SideWallCorrection()
    {
        if (ir_data[0] > ir_wall_base)
        {
            error_fl = ir_fl_base - (float)ir_data[0];
        }
        else
        {
            error_fl = 0;
        }

        if (ir_data[1] > ir_wall_base)
        {
            error_fr = ir_fr_base - (float)ir_data[1];
        }
        else
        {
            error_fr = 0;
        }
    }

    void Controller::PivotTurnRight90()
    {
        pivot_turn90.UpdateRef();
        ref_w = -pivot_turn90.GetRefVelocity();
        // u_v = pid_traslational_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_rotational_vel->Update(ref_w - cur_vel[1]) + ref_w / Kp_w;
        InputVelocity(u_v, u_w);
        // Logger();
        if (pivot_turn90.Finished())
        {
            // robot_dir_index = (robot_dir_index + 1) % 4;
            Brake();
            flag_controller = true;
        }
    }

    void Controller::PivotTurnLeft90()
    {
        pivot_turn90.UpdateRef();
        ref_w = pivot_turn90.GetRefVelocity();
        // u_v = pid_traslational_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_rotational_vel->Update(ref_w - cur_vel[1]) + ref_w / Kp_w;
        InputVelocity(u_v, u_w);
        // Logger();

        if (pivot_turn90.Finished())
        {
            Brake();
            flag_controller = true;
        }
    }

    void Controller::PivotTurn180()
    {
        pivot_turn180.UpdateRef();
        ref_w = pivot_turn180.GetRefVelocity();
        // u_v = pid_traslational_vel->Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_rotational_vel->Update(ref_w - cur_vel[1]) + ref_w / Kp_w;
        InputVelocity(u_v, u_w);
        // Logger();
        if (pivot_turn180.Finished())
        {
            Brake();
            flag_controller = true;
        }
    }

    void Controller::Turn()
    {
        slalom->UpdateRef();
        ref_pos = slalom->GetRefPosition();
        ref_vel = slalom->GetRefVelocity();
        ref_acc = slalom->GetRefAcceleration();

        kanayama->UpdateRef(ref_pos, ref_vel);
        ref_vel = kanayama->CalcInput(cur_pos);
        u_v = pid_traslational_vel->Update(ref_vel[0] - cur_vel[0]) + ref_vel[0] / Kp_v;
        u_w = pid_rotational_vel->Update(ref_vel[1] - cur_vel[1]) + ref_vel[1] / Kp_w;

        // dynamic_feedback->UpdateRef(ref_pos, ref_vel, ref_acc);
        // ref_vel = dynamic_feedback->CalcInput(cur_pos, cur_vel);
        // u_v = pid_traslational_vel->Update(ref_vel[0] - cur_vel[0]) + ref_vel[0] / Kp_v;
        // u_w = pid_rotational_vel->Update(ref_vel[1] - cur_vel[1]) + ref_vel[1] / Kp_w;

        InputVelocity(u_v, u_w);
        // Logger();
        if (slalom->Finished())
        {
            flag_controller = true;
        }
    }

    void Controller::Acceleration()
    {
        SideWallCorrection();
        acc->UpdateRef();
        ref_pos[0] = acc->GetRefPosition();
        ref_pos[1] = 0.0;
        ref_pos[2] = 0.0;
        ref_vel[0] = acc->GetRefVelocity();
        ref_vel[1] = 0.0;
        ref_acc[0] = acc->GetRefAcceleration();
        ref_acc[1] = 0.0;
        // kanayama->UpdateRef(ref_pos, ref_vel);
        // ref_vel = kanayama->CalcInput(cur_pos);
        // Logger();
        u_v = pid_traslational_vel->Update(ref_vel[0] - cur_vel[0]) + (Tp1_v * ref_acc[0] + ref_vel[0]) / Kp_v;
        // u_w = pid_rotational_vel->Update(-cur_vel[1]);
        u_w = pid_ir_sensor_side->Update(error_fl - error_fr) + pid_angle->Update(-cur_pos[2]);
        InputVelocity(u_v, u_w);
        if (acc->Finished())
        {
            flag_controller = true;
        }
    }

    void Controller::GoStraight(float ref_l)
    {
        if (l < ref_l)
        {
            SideWallCorrection();
            u_v = pid_traslational_vel->Update(ref_v - cur_vel[0]) + ref_v / Kp_v;
            u_w = pid_ir_sensor_side->Update(error_fl - error_fr) + pid_angle->Update(-cur_pos[2]);
            // u_w = pid_angle->Update(-cur_pos[2]);
            InputVelocity(u_v, u_w);
        }
        else
        {
            flag_controller = true;
        }
    }

    void Controller::Back(int time)
    {
        if (cnt < time)
        {
            InputVelocity(-0.5, 0.0);
            cnt++;
        }
        else
        {
            flag_controller = true;
            odom->ResetTheta();
            // base_theta = 0;
        }
    }

    void Controller::Wait(int time)
    {
        if (cnt < time)
        {
            motor.Brake();
            cnt++;
        }
        else
            flag_controller = true;
    }

    void Controller::FrontWallCorrection(const std::vector<uint32_t> &ir_value)
    {
        if (cnt < correction_time)
        {
            float error_sl = ir_sl_base - (float)ir_value[2];
            float error_sr = ir_sr_base - (float)ir_value[3];
            v_left = pid_ir_sensor_front_left->Update(error_sl);
            v_right = pid_ir_sensor_front_right->Update(error_sr);
            motor.Drive(v_left, v_right);
            cnt++;
        }
        else
        {
            flag_controller = true;
        }
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
        Acceleration(AccType::stop);
        FrontWallCorrection();
        PivotTurn(180);
        Back();
    }

    void Controller::Brake()
    {
        mode = stop;
        motor.Brake();
    }

    void Controller::InputVelocity(float input_v, float input_w)
    {
        v_left = input_v - input_w;
        v_right = input_v + input_w;
        motor.Drive(v_left, v_right);
    }

    bool Controller::GetFlag()
    {
        return flag_controller;
    }

    void Controller::Reset()
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
        mode = stop;

        flag_controller = false;
        index_log = 0;
        cnt = 0;
    }

    void Controller::ResetWallFlag()
    {
        flag_wall = false;
    }

    void Controller::MotorTest(float v_left, float v_right)
    {
        motor.Drive(v_left, v_right); // voltage [V]
    }

    // void Controller::Logger()
    // {
    //     log_x[index_log] = cur_pos[0];
    //     log_y[index_log] = cur_pos[1];
    //     log_theta[index_log] = cur_pos[2];
    //     // x[index_log] = ref_pos[0];
    //     // y[index_log] = ref_pos[1];
    //     // theta[index_log] = ref_pos[2];
    //     log_l[index_log] = l;
    //     log_v[index_log] = cur_vel[0];
    //     log_a[index_log] = acc_x;
    //     log_ref_l[index_log] = ref_pos[0];
    //     log_ref_v[index_log] = ref_vel[0];
    //     log_ref_a[index_log] = ref_acc[0];
    //     log_omega[index_log] = cur_vel[1];
    //     log_kanayama_v[index_log] = ref_vel[0];
    //     log_kanayama_w[index_log] = ref_vel[1];
    //     index_log++;
    // }

    void Controller::OutputLog()
    {
        // printf("%f, %f, %f, %f\n", cur_pos[0], cur_pos[1], cur_pos[2], l);
        // printf("%f, %f\n", cur_pos[2], cur_vel[1]);
        // printf("%f\n", l);
        // printf("%f, %f\n", u_v, u_w);
        printf("%f\n", acc_x);
        // for (int i = 0; i < ref_size; i++)
        // {
        // printf("%f, %f, %f, %f, %f\n", log_x[i], log_y[i], log_theta[i], log_kanayama_v[i], log_kanayama_w[i]);
        // printf("%f, %f, %f, %f, %f, %f, %f\n", log_x[i], log_y[i], log_theta[i], log_v[i], log_omega[i], log_kanayama_v[i], log_kanayama_w[i]);
        // printf("%f, %f\n", log_theta[i], log_omega[i]);
        // printf("%f, %f, %f, %f, %f, %f\n", log_l[i], log_v[i], log_a[i], log_ref_l[i], log_ref_v[i], log_ref_a[i]);
        // }
    }

    bool Controller::wallDataReady()
    {
        return flag_wall;
    }

    Direction Controller::getWallData(const std::vector<uint32_t> &ir_value)
    {
        Direction wall;

        int8_t robot_dir_index = 0;
        while (1)
        {
            if (robot_dir.byte == NORTH << robot_dir_index)
                break;
            robot_dir_index++;
        }

        if (ir_value[2] > ir_wall_base || ir_value[3] > ir_wall_base)
        {
            wall.byte |= NORTH << robot_dir_index;
        }

        if (ir_value[0] > ir_wall_base)
        {
            wall.byte |= NORTH << (robot_dir_index + 3) % 4;
        }

        if (ir_value[1] > ir_wall_base)
        {
            wall.byte |= NORTH << (robot_dir_index + 1) % 4;
        }

        prev_wall_cnt = wall.nWall();

        return wall;
    }

    IndexVec Controller::getRobotPosition()
    {
        // 絶対座標系で返す
        return robot_position;
    }

    void Controller::UpdatePos(const Direction &dir)
    {
        if (NORTH == dir.byte)
        {
            robot_position += IndexVec::vecNorth;
        }
        else if (SOUTH == dir.byte)
        {
            robot_position += IndexVec::vecSouth;
        }
        else if (EAST == dir.byte)
        {
            robot_position += IndexVec::vecEast;
        }
        else if (WEST == dir.byte)
        {
            robot_position += IndexVec::vecWest;
        }
    }

    void Controller::UpdateDir(const Direction &dir)
    {
        robot_dir = dir;
    }

    void Controller::robotMove()
    {
        switch (mode)
        {
        case forward:
            GoStraight(FORWARD_LENGTH2);
            break;
        case acc_curve:
            Acceleration();
            break;
        case turn:
            Turn();
            break;
        case pivot_turn_right_90:
            PivotTurnRight90();
            break;
        case pivot_turn_left_90:
            PivotTurnLeft90();
            break;
        case pivot_turn_180:
            PivotTurn180();
            break;
        case front_wall_correction:
            FrontWallCorrection(ir_data);
            break;
        case back:
            Back(back_time);
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

    void Controller::robotMove(const Direction &dir)
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
        // 直進
        if (dir_diff == 0)
        {
            // GoStraight(ir_data);
            Acceleration(AccType::forward1);
        }
        // 右
        else if (dir_diff == 1 || dir_diff == -3)
        {
            Turn(-90);
        }
        // 左
        else if (dir_diff == -1 || dir_diff == 3)
        {
            Turn(90);
        }
        // 180度ターン
        else
        {
            // 袋小路
            if (prev_wall_cnt == 3)
            {
                BlindAlley();
            }
            else
            {
                PivotTurn180();
            }
        }
        flag_wall = true;
    }

    void Controller::robotMove2(const Direction &dir)
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
        // 直進
        if (dir_diff == 0)
        {
            Acceleration(AccType::forward1);
        }
        // 右
        else if (dir_diff == 1 || dir_diff == -3)
        {
            Acceleration(AccType::forward_half);
            if (ir_data[2] > ir_wall_base && ir_data[3] > ir_wall_base)
                FrontWallCorrection();
            PivotTurn(-90);
            Acceleration(AccType::forward_half);
        }
        // 左
        else if (dir_diff == -1 || dir_diff == 3)
        {
            Acceleration(AccType::forward_half);
            if (ir_data[2] > ir_wall_base && ir_data[3] > ir_wall_base)
                FrontWallCorrection();
            PivotTurn(90);
            Acceleration(AccType::forward_half);
        }
        // 180度ターン
        else
        {
            // 袋小路
            if (prev_wall_cnt == 3)
            {
                BlindAlley();
            }
            else
            {
                Acceleration(AccType::stop);
                PivotTurn180();
                Acceleration(AccType::stop);
            }
        }
        flag_wall = true;
    }

    void Controller::robotMove(const Operation &op)
    {
        switch (op.op)
        {
        case Operation::FORWARD:
            // Acceleration(AccType::forward1);
            GoStraight();
            break;

        case Operation::TURN_LEFT90:
            Turn(90);
            break;

        case Operation::TURN_RIGHT90:
            Turn(-90);
            break;

        case Operation::STOP:
            Brake();
            break;
        default:
            break;
        }
    }

    void Controller::robotMove2(const Operation &op)
    {
        switch (op.op)
        {
        case Operation::FORWARD:
            Acceleration(AccType::forward1);
            break;

        case Operation::TURN_LEFT90:
            if (ir_data[2] > ir_wall_base && ir_data[3] > ir_wall_base)
                FrontWallCorrection();
            PivotTurn(90);
            Acceleration(AccType::forward1);
            break;

        case Operation::TURN_RIGHT90:
            if (ir_data[2] > ir_wall_base && ir_data[3] > ir_wall_base)
                FrontWallCorrection();
            PivotTurn(-90);
            Acceleration(AccType::forward1);
            break;

        case Operation::STOP:
            Brake();
            break;
        default:
            break;
        }
    }
}