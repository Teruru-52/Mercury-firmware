#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "main.h"
#include "hardware/motor.h"
#include "odometory.h"
#include "controller/pid_controller.h"
#include "controller/kanayama.h"
#include "dynamic_feedback.h"
#include "trajectory.h"
#include "state.h"
#include "Operation.h"
#include "Maze.h"
#include "Agent.h"

#define FORWARD_LENGTH1 0.138
#define FORWARD_LENGTH2 0.18
#define FORWARD_LENGTH3 0.09
#define FORWARD_LENGTH4 0.54

using AccType = trajectory::Acceleration::AccType;
using namespace hardware;

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(undercarriage::Odometory *odom,
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
                   hardware::IR_Base *ir_base);

        typedef enum
        {
            forward,
            acc_curve,
            turn,
            pivot_turn_right_90,
            pivot_turn_left_90,
            pivot_turn_180,
            front_wall_correction,
            back,
            stop,
            wait
        } Mode;

        void InitializeOdometory();
        void UpdateBatteryVoltage(float bat_vol);
        void UpdateOdometory();
        bool ErrorFlag();
        void ResetOdometory();
        int16_t GetPulse();
        void UpdateIMU();
        // void SetBase();
        void SetIRdata(const IR_Value &ir_value);
        void SetTrajectoryMode(int mode = 1);

        void PivotTurn(int angle);
        void Turn(int angle);
        void Acceleration(const AccType &acc_type);
        void GoStraight();
        void FrontWallCorrection();
        void Back();
        void Wait();

        void PartyTrick();
        void SideWallCorrection();
        void PivotTurnRight90();
        void PivotTurnLeft90();
        void PivotTurn180();
        void Turn();
        void Acceleration();
        void GoStraight(float ref_l);
        void Back(int time);
        void Wait(int time);
        void FrontWallCorrection(const IR_Value &ir_value);
        void BlindAlley();
        void StartMove();
        void InitializePosition();
        void Brake();
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        void Reset();
        void ResetWallFlag();
        void MotorTest(float v_left, float v_right);
        void Logger();
        void OutputLog();

        bool wallDataReady();
        Direction getWallData(const IR_Value &ir_value);
        void UpdatePos(const Direction &dir);
        void UpdateDir(const Direction &dir);
        IndexVec getRobotPosition();
        void robotMove();
        void robotMove(const Direction &dir);
        void robotMove2(const Direction &dir);
        void robotMove(const Operation &op);
        void robotMove2(const Operation &op);

    private:
        undercarriage::Odometory *odom;
        hardware::Motor motor;
        PID *pid_angle;
        PID *pid_rotational_vel;
        PID *pid_traslational_vel;
        PID *pid_ir_sensor_front_left;
        PID *pid_ir_sensor_front_right;
        PID *pid_ir_sensor_side;
        undercarriage::Kanayama *kanayama;
        undercarriage::Dynamic_Feedback *dynamic_feedback;
        trajectory::Slalom *slalom;
        trajectory::Acceleration *acc;
        trajectory::PivotTurn180 pivot_turn180;
        trajectory::PivotTurn90 pivot_turn90;
        Mode mode;
        hardware::IR_Base *ir_base;

        float v_left;
        float v_right;
        float u_w;
        float u_v;
        int ref_size;
        IR_Value ir_value;
        ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
        ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
        const float Tp1_w = 31.83;
        const float Kp_w = 144.2;
        const float Tp1_v = 0.032;
        const float Kp_v = 0.784493;
        float ref_v = 0.186825;
        float ref_w;
        const int back_time = 400;       // ms
        const int correction_time = 500; // ms
        const int wait_time = 200;       // ms
        bool flag_controller;
        bool flag_wall;
        bool flag_safety;
        int cnt;
        int index_log;
        float l;
        ctrl::Pose cur_pos{0, 0, 0};
        ctrl::Pose cur_vel{0, 0, 0};
        float acc_x;
        // float base_theta;
        float error_fl;
        float error_fr;
        // float *log_x;
        // float *log_y;
        // float *log_theta;
        // float *log_l;
        // float *log_v;
        // float *log_a;
        // float *log_ref_l;
        // float *log_ref_v;
        // float *log_ref_a;
        // float *log_omega;
        // float *log_kanayama_v;
        // float *log_kanayama_w;

        int prev_wall_cnt = 0;
        int8_t dir_diff;
        IndexVec robot_position;
        Direction robot_dir;
    };
} // namespace undercarriage

#endif //  CONTROLLER_H_