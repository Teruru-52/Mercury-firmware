#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "main.h"
// #include "instance.h"
#include <vector>
#include "hardware/motor.h"
#include "odometory.h"
#include "controller/pid_controller.h"
#include "controller/kanayama.h"
#include "trajectory.h"
#include "Operation.h"
#include "Maze.h"
#include "Agent.h"

#define FORWARD_LENGTH1 0.138
#define FORWARD_LENGTH2 0.18
#define FORWARD_LENGTH3 0.09
#define FORWARD_LENGTH4 0.54

using AccType = trajectory::Acceleration::AccType;

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(undercarriage::Odometory *odom,
                   PID *pid_angle,
                   PID *pid_rotational_vel,
                   PID *pid_traslational_vel,
                   PID *pid_ir_sensor_front,
                   PID *pid_ir_sensor_side,
                   undercarriage::Kanayama *kanayama,
                   trajectory::Slalom *slalom,
                   trajectory::Acceleration *acc);

        typedef enum
        {
            FORWARD,
            ACC_CURVE,
            TURN,
            PIVOT_TURN_RIGHT90,
            PIVOT_TURN_LEFT90,
            PIVOT_TURN180,
            FRONT_WALL_CORRECTION,
            BACK,
            STOP
        } Mode;

        void InitializeOdometory();
        void UpdateBatteryVoltage(float bat_vol);
        void UpdateOdometory();
        void ResetOdometory();
        int16_t GetPulse();
        void UpdateIMU();
        void SetBase();
        void SetTrajectoryMode(int mode = 1);

        void PivotTurn(int angle);
        void Turn(float angle);
        void Acceleration(const AccType &acc_type);
        void FrontWallCorrection();
        void Back();

        void PartyTrick();
        void SideWallCorrection(const std::vector<uint32_t> &ir_data);
        void PivotTurnRight90();
        void PivotTurnLeft90();
        void PivotTurn180();
        void Turn();
        void Acceleration(const std::vector<uint32_t> &ir_data);
        // void GoStraight(const std::vector<uint32_t> &ir_data);
        void Back(int time);
        void FrontWallCorrection(const std::vector<uint32_t> &ir_data);
        void BlindAlley();
        void StartMove();
        void InitializePosition(const std::vector<uint32_t> &ir_data);
        void Brake();
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        void Reset();
        void ResetWallFlag();
        void MotorTest(float v_left, float v_right);
        void Logger();
        void OutputLog();

        bool wallDataReady();
        Direction getWallData(const std::vector<uint32_t> &ir_data);
        void UpdatePos(const Direction &dir);
        void UpdateDir(const Direction &dir);
        IndexVec getRobotPosition();
        void robotMove(const std::vector<uint32_t> &ir_data);
        void robotMove(const Operation &op, const std::vector<uint32_t> &ir_data);
        void robotMove(const Direction &dir, const std::vector<uint32_t> &ir_data);
        void robotMove2(const Direction &dir, const std::vector<uint32_t> &ir_data);

    private:
        undercarriage::Odometory *odom;
        hardware::Motor motor;
        PID *pid_angle;
        PID *pid_rotational_vel;
        PID *pid_traslational_vel;
        PID *pid_ir_sensor_front;
        PID *pid_ir_sensor_side;
        undercarriage::Kanayama *kanayama;
        trajectory::Slalom *slalom;
        trajectory::Acceleration *acc;
        trajectory::PivotTurn180 pivot_turn180;
        trajectory::PivotTurn90 pivot_turn90;
        Mode mode;

        float v_left;
        float v_right;
        float u_w;
        float u_v;
        int ref_size;
        std::vector<float> ref_pos{0, 0, 0};
        std::vector<float> ref_vel{0, 0};
        std::vector<float> ref_acc{0, 0};
        const float Tp1_w = 31.83;
        const float Kp_w = 144.2;
        const float Tp1_v = 0.032;
        const float Kp_v = 0.784493;
        // float ref_v = 0.2132397;
        float ref_w;
        float ref_l;
        const float ir_fl_base = 2220;
        const float ir_fr_base = 2280;
        const float ir_sl_base = 3400;
        const float ir_sr_base = 3400;
        const float ir_wall_base = 2180;
        bool flag_controller;
        bool flag_wall;
        int cnt;
        int index_log;
        float l;
        std::vector<float> cur_pos{0, 0, 0};
        std::vector<float> def_pos{0, 0, 0};
        std::vector<float> cur_vel{0, 0};
        // float base_theta;
        float error_fl;
        float error_fr;
        float *log_x;
        float *log_y;
        float *log_theta;
        float *log_l;
        float *log_v;
        float *log_ref_l;
        float *log_ref_v;
        float *log_ref_a;
        float *log_omega;
        float *log_kanayama_v;
        float *log_kanayama_w;

        int prev_wall_cnt = 0;
        int8_t dir_diff;
        IndexVec robot_position;
        Direction robot_dir;
        // int8_t robot_dir_index = 0;
    };
} // namespace undercarriage

#endif //  CONTROLLER_H_