#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "main.h"
#include <vector>
#include "../hardware/motor.h"
#include "../odometory.h"
#include "../controller/pid_controller.h"
#include "../controller/kanayama.h"
#include "../state.h"
#include "Operation.h"
#include "Maze.h"
#include "Agent.h"

#define FORWARD_LENGTH1 0.144
#define FORWARD_LENGTH2 0.18
#define FORWARD_LENGTH3 0.03 // 0.09

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(float sampling_period, float control_period);

        void InitializeOdometory();
        void UpdateBatteryVoltage(float bat_vol);
        void UpdateOdometory();
        void UpdateIMU();
        void SetBase();
        void PartyTrick();
        void PivotTurnRight90();
        void PivotTurnLeft90();
        void PivotTurn180();
        void KanayamaTurnLeft90();
        void KanayamaTurnRight90();
        void GoStraight(const std::vector<uint32_t> &ir_data);
        void Back();
        void FrontWallCorrection(const std::vector<uint32_t> &ir_data);
        void BlindAlley(const std::vector<uint32_t> &ir_data);
        void StartMove(const std::vector<uint32_t> &ir_data);
        void Brake();
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        void Reset();
        void MotorTest(float v_left, float v_right);
        void Logger();
        void OutputLog();

        Direction getWallData(const std::vector<uint32_t> &ir_data);
        void UpdatePos(const Direction &dir);
        IndexVec getRobotPosition();
        void robotMove(const Operation &op, const std::vector<uint32_t> &ir_data);
        void robotMove(const Direction &dir, const std::vector<uint32_t> &ir_data);
        void robotMove(const State::Mode &mode, const std::vector<uint32_t> &ir_data);

    private:
        undercarriage::Odometory odom;
        hardware::Motor motor;
        PID pid_angle;
        PID pid_rotational_vel;
        PID pid_traslational_vel;
        PID pid_ir_sensor_front;
        PID pid_ir_sensor_side;
        undercarriage::Kanayama kanayama;
        trajectory::PivotTurn180 pivot_turn180;
        trajectory::PivotTurn90 pivot_turn90;
        State state;

        float v_left;
        float v_right;
        float u_w;
        float u_v;
        int ref_size;
        std::vector<float> ref_vel{0, 0};
        const float Tp1_w = 31.83;
        const float Kp_w = 144.2;
        // const float Tp1_v = 0.18577;
        const float Tp1_v = 0.032;
        const float Kp_v = 0.79586;
        // const float ref_v = 0.5064989;
        const float ref_v = 0.2132397;
        float ref_w;
        float ref_l;
        float ref_theta;
        const float ir_fl_base = 2220;
        const float ir_fr_base = 2280;
        const float ir_sl_base = 3400;
        const float ir_sr_base = 3400;
        bool flag;
        int cnt;
        int index_log;
        float l;
        std::vector<float> cur_pos{0, 0, 0};
        std::vector<float> def_pos{0, 0, 0};
        std::vector<float> cur_vel{0, 0};
        float base_theta;
        float *x;
        float *y;
        float *theta;
        float *v;
        float *omega;
        float *kanayama_v;
        float *kanayama_w;

        int prev_wall_cnt = 0;
        IndexVec robot_position;
        Direction robot_dir;
        // int8_t robot_dir_index = 0;
    };
} // namespace undercarriage

#endif //  CONTROLLER_HPP_