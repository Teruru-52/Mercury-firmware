#include "../../Inc/controller/controller.h"

namespace undercarriage
{
    Controller::Controller(float control_period)
        : pid_angle(5.0, 2.0, 0.05, 0.0, control_period),
          pid_rotational_vel(1.1976, 85.1838, -0.00099, 0.0039227, control_period),
          pid_traslational_vel(6.8176, 82.0249, -0.033349, 0.023191, control_period),
          pid_ir_sensor_left(1.0, 0.0, 0.0, 0.0, control_period),
          pid_ir_sensor_right(1.0, 0.0, 0.0, 0.0, control_period),
          kanayama(1.0, 1.0, 10.0),
          flag(false),
          index_log(0)
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

    void Controller::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Controller::robotMove(const Operation &op, const std::vector<float> &cur_pos, const std::vector<float> &cur_vel, const std::vector<uint32_t> &ir_data, float l)
    {
        flag = false;
        switch (op.op)
        {
        case Operation::FORWARD:
            GoStraight(cur_pos, cur_vel, ir_data, l);
            break;
        case Operation::TURN_LEFT90:
            KanayamaTurnLeft90(cur_pos, cur_vel);
            break;
        case Operation::TURN_RIGHT90:
            KanayamaTurnRight90(cur_pos, cur_vel);
            break;
        case Operation::STOP:
            motor.Brake();
            break;
        default:
            break;
        }
    }

    void Controller::robotMove(const Direction &dir)
    {
    }

    void Controller::robotMove(const State::Mode &mode, const std::vector<float> &cur_pos, const std::vector<float> &cur_vel, const std::vector<uint32_t> &ir_data, float l)
    {
        flag = false;
        switch (mode)
        {
        case State::FORWARD:
            GoStraight(cur_pos, cur_vel, ir_data, l);
            break;
        case State::TURN_LEFT90:
            KanayamaTurnLeft90(cur_pos, cur_vel);
            break;
        case State::PIVOT_TURN90:
            PivotTurn90(cur_pos, cur_vel);
            break;
        case State::PIVOT_TURN180:
            PivotTurn180(cur_pos, cur_vel);
            break;
        default:
            break;
        }
    }

    void Controller::SetBase(const std::vector<float> &cur_pos, float l)
    {
        base_cur_pos[0] = cur_pos[0];
        base_cur_pos[1] = cur_pos[1];
        base_cur_pos[2] = cur_pos[2];
        base_l = l;
    }

    void Controller::PartyTrick(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        // u_v = pid_traslational_vel.Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_angle.Update(-cur_pos[2]) + pid_rotational_vel.Update(-cur_vel[1]);
        InputVelocity(u_v, u_w);
    }

    void Controller::PivotTurn90(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        if (pivot_turn90.GetFlag())
        {
            pivot_turn90.UpdateRef();
            ref_w = pivot_turn90.GetRefVelocity();
            u_v = 0;
            u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1_w * ref_w / Kp_w;
            InputVelocity(u_v, u_w);

            theta[index_log] = cur_pos[2];
            omega[index_log] = cur_vel[1];
            index_log++;
        }
        else
        {
            motor.Brake();
            pivot_turn90.ResetTrajectoryIndex();
            pivot_turn90.ResetFlag();
            flag = true;
        }
    }

    void Controller::PivotTurn180(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        if (pivot_turn180.GetFlag())
        {
            pivot_turn180.UpdateRef();
            ref_w = pivot_turn180.GetRefVelocity();
            u_v = 0;
            u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1_w * ref_w / Kp_w;
            InputVelocity(u_v, u_w);

            theta[index_log] = cur_pos[2];
            omega[index_log] = cur_vel[1];
            index_log++;
        }
        else
        {
            motor.Brake();
            pivot_turn180.ResetTrajectoryIndex();
            flag = true;
            pivot_turn180.ResetFlag();
        }
    }

    void Controller::KanayamaTurnLeft90(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        if (kanayama.GetFlag())
        {
            kanayama.UpdateRef();
            for (int i = 0; i < 3; i++)
            {
                def_cur_pos[i] = cur_pos[i] - base_cur_pos[i];
            }
            ref_vel = kanayama.CalcInput(def_cur_pos);
            u_v = pid_traslational_vel.Update(ref_vel[0] - cur_vel[0]) + Tp1_v * ref_vel[0] / Kp_v;
            u_w = pid_rotational_vel.Update(ref_vel[1] - cur_vel[1]) + Tp1_w * ref_vel[1] / Kp_w;
            InputVelocity(u_v, u_w);

            x[index_log] = def_cur_pos[0];
            y[index_log] = def_cur_pos[1];
            theta[index_log] = def_cur_pos[2];
            v[index_log] = cur_vel[0];
            omega[index_log] = cur_vel[1];
            kanayama_v[index_log] = ref_vel[0];
            kanayama_w[index_log] = ref_vel[1];
            index_log++;
        }
        else
        {
            motor.Brake();
            kanayama.ResetTrajectoryIndex();
            flag = true;
        }
    }

    void Controller::KanayamaTurnRight90(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        if (kanayama.GetFlag())
        {
            kanayama.UpdateRef2();
            for (int i = 0; i < 3; i++)
            {
                def_cur_pos[i] = cur_pos[i] - base_cur_pos[i];
            }
            ref_vel = kanayama.CalcInput(def_cur_pos);
            u_v = pid_traslational_vel.Update(ref_vel[0] - cur_vel[0]) + Tp1_v * ref_vel[0] / Kp_v;
            u_w = pid_rotational_vel.Update(ref_vel[1] - cur_vel[1]) + Tp1_w * ref_vel[1] / Kp_w;
            InputVelocity(u_v, u_w);

            x[index_log] = def_cur_pos[0];
            y[index_log] = def_cur_pos[1];
            theta[index_log] = def_cur_pos[2];
            v[index_log] = cur_vel[0];
            omega[index_log] = cur_vel[1];
            kanayama_v[index_log] = ref_vel[0];
            kanayama_w[index_log] = ref_vel[1];
            index_log++;
        }
        else
        {
            motor.Brake();
            kanayama.ResetTrajectoryIndex();
            flag = true;
        }
    }

    void Controller::GoStraight(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel, const std::vector<uint32_t> &ir_data, float l)
    {
        def_l = l - base_l;
        if (def_l >= FORWARD_LENGTH1)
        {
            flag = true;
        }
        u_v = pid_traslational_vel.Update(ref_v - cur_vel[0]) + Tp1_v * ref_v / Kp_v;
        // u_w = pid_ir_sensor_left.Update((float)(ir_straight - ir_data[2])) + pid_ir_sensor_right.Update((float)(ir_straight - ir_data[3])) + pid_angle.Update(-(cur_pos[2] - base_cur_pos[2]));
        u_w = pid_angle.Update(-cur_pos[2]);
        InputVelocity(u_v, u_w);
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

    void Controller::ResetFlag()
    {
        flag = false;
        index_log = 0;
    }

    float Controller::GetInput()
    {
        // return u_v;
        return u_w;
    }

    void Controller::MotorTest(float v_left, float v_right)
    {
        motor.Drive(v_left, v_right); // voltage [V]
    }

    void Controller::OutputLog()
    {
        // printf("%f\n", def_l);
        // printf("%f, %f\n", u_v, u_w);
        for (int i = 0; i < ref_size; i++)
        {
            printf("%f, %f, %f, %f, %f, %f, %f\n", x[i], y[i], theta[i], v[i], omega[i], kanayama_v[i], kanayama_w[i]);
            // printf("%f, %f\n", theta[i], omega[i]);
        }
    }
}