#include "../../Inc/controller/controller.h"

namespace undercarriage
{
    Controller::Controller(float sampling_period, float control_period)
        : odom(sampling_period),
          pid_angle(4.0, 0.0, 0.0, 0.0, control_period),
          pid_rotational_vel(1.1976, 85.1838, -0.00099, 0.0039227, control_period),
          //   pid_traslational_vel(6.8176, 82.0249, -0.033349, 0.023191, control_period),
          pid_traslational_vel(2.0, 82.0249, -0.0003, 0.002, control_period),
          pid_ir_sensor_front(0.001, 0.001, 0.0, 0.0, control_period),
          pid_ir_sensor_side(0.001, 0.001, 0.0, 0.0, control_period),
          kanayama(3.0, 3.0, 10.0),
          state(State::FORWARD),
          ref_l(FORWARD_LENGTH1),
          ref_theta(0),
          flag(false),
          cnt(0),
          index_log(0),
          base_theta(0),
          robot_position(0, 0),
          robot_dir(NORTH)
    {
        // ref_size = pivot_turn180.GetRefSize();
        // ref_size = pivot_turn90.GetRefSize();
        ref_size = 284; // kanayama ref_time = 0.151678
        x = new float[ref_size];
        y = new float[ref_size];
        theta = new float[ref_size];
        v = new float[ref_size];
        omega = new float[ref_size];
        kanayama_v = new float[ref_size];
        kanayama_w = new float[ref_size];
    }

    void Controller::InitializeOdometory()
    {
        odom.Initialize();
    }

    void Controller::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Controller::UpdateOdometory()
    {
        odom.Update();
        l = odom.GetLength();
        cur_pos = odom.GetPosition();
        cur_vel = odom.GetVelocity();
    }

    void Controller::UpdateIMU()
    {
        odom.UpdateIMU();
    }

    void Controller::SetBase()
    {
        base_theta = cur_pos[2];
    }

    void Controller::PartyTrick()
    {
        // u_v = pid_traslational_vel.Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_angle.Update(-cur_pos[2]) + pid_rotational_vel.Update(-cur_vel[1]);
        InputVelocity(u_v, u_w);
    }

    void Controller::PivotTurnRight90()
    {
        if (pivot_turn90.GetFlag())
        {
            pivot_turn90.UpdateRef();
            ref_w = -pivot_turn90.GetRefVelocity();
            u_v = 0;
            u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1_w * ref_w / Kp_w;
            InputVelocity(u_v, u_w);

            // Logger();
        }
        else
        {
            // robot_dir_index = (robot_dir_index + 1) % 4;
            printf("finish\n");
            Brake();
            pivot_turn90.ResetTrajectoryIndex();
            pivot_turn90.ResetFlag();
            flag = true;
        }
    }

    void Controller::PivotTurnLeft90()
    {
        if (pivot_turn90.GetFlag())
        {
            pivot_turn90.UpdateRef();
            ref_w = pivot_turn90.GetRefVelocity();
            u_v = 0;
            u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1_w * ref_w / Kp_w;
            InputVelocity(u_v, u_w);

            // Logger();
        }
        else
        {
            // robot_dir_index = (robot_dir_index + 3) % 4;
            Brake();
            pivot_turn90.ResetTrajectoryIndex();
            pivot_turn90.ResetFlag();
            flag = true;
        }
    }

    void Controller::PivotTurn180()
    {
        if (pivot_turn180.GetFlag())
        {
            pivot_turn180.UpdateRef();
            ref_w = pivot_turn180.GetRefVelocity();
            u_v = 0;
            u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1_w * ref_w / Kp_w;
            InputVelocity(u_v, u_w);

            // Logger();
        }
        else
        {
            // robot_dir_index = (robot_dir_index + 2) % 4;
            Brake();
            pivot_turn180.ResetTrajectoryIndex();
            pivot_turn180.ResetFlag();
            flag = true;
        }
    }

    void Controller::KanayamaTurnLeft90()
    {
        if (kanayama.GetFlag())
        {
            def_pos[0] = cur_pos[0];
            def_pos[1] = cur_pos[1];
            def_pos[2] = cur_pos[2] - base_theta;
            kanayama.UpdateRef();
            ref_vel = kanayama.CalcInput(def_pos);
            u_v = pid_traslational_vel.Update(ref_vel[0] - cur_vel[0]) + Tp1_v * ref_vel[0] / Kp_v;
            u_w = pid_rotational_vel.Update(ref_vel[1] - cur_vel[1]) + Tp1_w * ref_vel[1] / Kp_w;
            InputVelocity(u_v, u_w);

            Logger();
        }
        else
        {
            // robot_dir_index = (robot_dir_index + 3) % 4;
            kanayama.Reset();
            ref_theta += M_PI_2;
            flag = true;
        }
    }

    void Controller::KanayamaTurnRight90()
    {
        if (kanayama.GetFlag())
        {
            def_pos[0] = cur_pos[0];
            def_pos[1] = cur_pos[1];
            def_pos[2] = cur_pos[2] - base_theta;
            kanayama.UpdateRef2();
            ref_vel = kanayama.CalcInput(def_pos);
            u_v = pid_traslational_vel.Update(ref_vel[0] - cur_vel[0]) + Tp1_v * ref_vel[0] / Kp_v;
            u_w = pid_rotational_vel.Update(ref_vel[1] - cur_vel[1]) + Tp1_w * ref_vel[1] / Kp_w;
            InputVelocity(u_v, u_w);

            Logger();
        }
        else
        {
            // robot_dir_index = (robot_dir_index + 1) % 4;
            kanayama.Reset();
            ref_theta -= M_PI_2;
            flag = true;
        }
    }

    void Controller::GoStraight(const std::vector<uint32_t> &ir_data)
    {
        float error_fl;
        float error_fr;
        if (l < ref_l)
        {
            if (ir_data[0] > 2150)
            {
                error_fl = ir_fl_base - (float)ir_data[0];
            }
            else
            {
                error_fl = 0;
            }

            if (ir_data[1] > 2150)
            {
                error_fr = ir_fr_base - (float)ir_data[1];
            }
            else
            {
                error_fr = 0;
            }

            u_v = pid_traslational_vel.Update(ref_v - cur_vel[0]) + Tp1_v * ref_v / Kp_v;
            u_w = pid_ir_sensor_side.Update(error_fl - error_fr) + pid_angle.Update(ref_theta - cur_pos[2]);
            // u_w = pid_angle.Update(ref_theta - cur_pos[2]);
            InputVelocity(u_v, u_w);
        }
        else
        {
            // if (ref_l != FORWARD_LENGTH2)
            // {
            //     ref_l = FORWARD_LENGTH2;
            // }
            flag = true;
        }
    }

    void Controller::Back()
    {
        if (cnt < 500)
        {
            InputVelocity(-0.5, 0.0);
            cnt++;
        }
        else
        {
            ref_l = FORWARD_LENGTH1;
            flag = true;
            odom.ResetTheta();
            ref_theta = 0;
            base_theta = 0;
        }
    }

    void Controller::FrontWallCorrection(const std::vector<uint32_t> &ir_data)
    {
        if (cnt < 1000)
        {
            float error_sl = ir_sl_base - (float)ir_data[2];
            float error_sr = ir_sr_base - (float)ir_data[3];
            v_left = pid_ir_sensor_front.Update(error_sl);
            v_right = pid_ir_sensor_front.Update(error_sr);
            motor.Drive(v_left, v_right);
            cnt++;
        }
        else
        {
            flag = true;
        }
    }

    void Controller::BlindAlley(const std::vector<uint32_t> &ir_data)
    {
        if (state.mode == State::FORWARD)
        {
            ref_l = FORWARD_LENGTH3;
            GoStraight(ir_data);
            if (GetFlag())
            {
                Reset();
                state.mode = State::PIVOT_TURN_RIGHT90;
            }
        }
        if (state.mode == State::PIVOT_TURN_RIGHT90)
        {
            PivotTurnRight90();
            if (GetFlag())
            {
                Reset();
                state.mode = State::FRONT_WALL_CORRECTION;
            }
        }
        if (state.mode == State::FRONT_WALL_CORRECTION)
        {
            FrontWallCorrection(ir_data);
            if (GetFlag())
            {
                Reset();
                state.mode = State::PIVOT_TURN_RIGHT90_2;
            }
        }
        if (state.mode == State::PIVOT_TURN_RIGHT90_2)
        {
            PivotTurnRight90();
            if (GetFlag())
            {
                Reset();
                state.mode = State::BACK;
            }
        }
        if (state.mode == State::BACK)
        {
            Back();
            if (GetFlag())
            {
                Reset();
                state.mode = State::FORWARD2;
            }
        }
        if (state.mode == State::FORWARD2)
        {
            ref_l = FORWARD_LENGTH1;
            GoStraight(ir_data);
            if (GetFlag())
            {
                state.mode = State::TURN_RIGHT90;
            }
        }
    }

    void Controller::StartMove(const std::vector<uint32_t> &ir_data)
    {
        if (state.mode == State::PIVOT_TURN_RIGHT90)
        {
            PivotTurnRight90();
            if (GetFlag())
            {
                Reset();
                state.mode = State::FRONT_WALL_CORRECTION;
            }
        }
        if (state.mode == State::FRONT_WALL_CORRECTION)
        {
            FrontWallCorrection(ir_data);
            if (GetFlag())
            {
                Reset();
                state.mode = State::PIVOT_TURN_LEFT90;
            }
        }
        if (state.mode == State::PIVOT_TURN_LEFT90)
        {
            PivotTurnLeft90();
            if (GetFlag())
            {
                Reset();
                state.mode = State::BACK;
            }
        }
        if (state.mode == State::BACK)
        {
            Back();
            if (GetFlag())
            {
                Reset();
                state.mode = State::FORWARD;
            }
        }
        if (state.mode == State::FORWARD)
        {
            GoStraight(ir_data);
            if (GetFlag())
            {
                state.mode = State::PIVOT_TURN_RIGHT90;
            }
        }
    }

    void Controller::Brake()
    {
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
        return flag;
    }

    void Controller::Reset()
    {
        flag = false;
        index_log = 0;
        cnt = 0;
        odom.Reset();
    }

    void Controller::MotorTest(float v_left, float v_right)
    {
        motor.Drive(v_left, v_right); // voltage [V]
    }

    void Controller::Logger()
    {
        x[index_log] = cur_pos[0];
        y[index_log] = cur_pos[1];
        theta[index_log] = cur_pos[2];
        v[index_log] = cur_vel[0];
        omega[index_log] = cur_vel[1];
        kanayama_v[index_log] = ref_vel[0];
        kanayama_w[index_log] = ref_vel[1];
        index_log++;
    }

    void Controller::OutputLog()
    {
        printf("%f, %f, %f, %f\n", cur_pos[0], cur_pos[1], cur_pos[2], l);
        // printf("%f, %f\n", cur_pos[2], cur_vel[1]);
        // printf("%f\n", l);
        // printf("%f, %f\n", u_v, u_w);
        //     for (int i = 0; i < ref_size; i++)
        //     {
        //         printf("%f, %f, %f, %f, %f, %f, %f\n", x[i], y[i], theta[i], v[i], omega[i], kanayama_v[i], kanayama_w[i]);
        //         // printf("%f, %f\n", theta[i], omega[i]);
        //     }
    }

    Direction Controller::getWallData(const std::vector<uint32_t> &ir_data)
    {
        Direction wall;

        int8_t robot_dir_index = 0;
        while (1)
        {
            if (robot_dir.byte == NORTH << robot_dir_index)
                break;
            robot_dir_index++;
        }

        if ((ir_data[0] + ir_data[1]) * 0.5 > 2500)
        {
            wall.byte |= NORTH << robot_dir_index;
        }

        if (ir_data[2] > 2300)
        {
            wall.byte |= NORTH << (robot_dir_index + 3) % 4;
        }

        if (ir_data[3] > 2300)
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
        robot_dir = dir;
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

    void Controller::robotMove(const Operation &op, const std::vector<uint32_t> &ir_data)
    {
        flag = false;
        switch (op.op)
        {
        case Operation::FORWARD:
            GoStraight(ir_data);
            break;
        case Operation::TURN_LEFT90:
            KanayamaTurnLeft90();
            break;
        case Operation::TURN_RIGHT90:
            KanayamaTurnRight90();
            break;
        case Operation::STOP:
            motor.Brake();
            break;
        default:
            break;
        }
    }

    void Controller::robotMove(const Direction &dir, const std::vector<uint32_t> &ir_data)
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

        int8_t dir_diff = next_dir_index - robot_dir_index;
        // 直進
        if (dir_diff == 0)
        {
            GoStraight(ir_data);
        }
        // 右
        else if (dir_diff == 1 || dir_diff == -3)
        {
            KanayamaTurnRight90();
        }
        // 左
        else if (dir_diff == -1 || dir_diff == 3)
        {
            KanayamaTurnLeft90();
        }
        // 180度ターン
        else
        {
            // 袋小路
            if (prev_wall_cnt == 3)
            {
                BlindAlley(ir_data);
            }
            else
            {
                PivotTurn180();
            }
        }
    }

    void Controller::robotMove(const State::Mode &mode, const std::vector<uint32_t> &ir_data)
    {
        flag = false;
        switch (mode)
        {
        case State::FORWARD:
            GoStraight(ir_data);
            break;
        case State::TURN_LEFT90:
            KanayamaTurnLeft90();
            break;
        case State::PIVOT_TURN_RIGHT90:
            PivotTurnRight90();
            break;
        case State::PIVOT_TURN180:
            PivotTurn180();
            break;
        case State::BACK:
            Back();
            break;
        default:
            break;
        }
    }

}
