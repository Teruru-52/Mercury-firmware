#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "main.h"
#include "hardware/motor.h"
#include "hardware/speaker.h"
#include "odometory.h"
#include "controller/pid_controller.h"
#include "controller/kanayama.h"
#include "dynamic_feedback.h"
#include "trajectory.h"
#include "m_identification.h"
#include "step_identification.h"
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
        Controller(Speaker *speaker,
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
                   trajectory::Velocity *velocity);

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
            m_iden,
            step_iden,
            party_trick,
            stop,
            wait
        } Mode;

        void InitializeOdometory() { odom->Initialize(); };
        int16_t GetPulseL() { return odom->GetPulseL(); };
        int16_t GetPulseR() { return odom->GetPulseR(); };

        void UpdateBatteryVoltage(float bat_vol);
        void UpdateOdometory();
        bool ErrorFlag();
        // void UpdateIMU() { odom->UpdateIMU(); };
        // void SetBase() { theta_base = cur_pos.th; };
        void SetIRdata(const IR_Value &ir_value);
        void SetTrajectoryMode(int mode = 1);

        void PivotTurn(int angle);
        void Turn(int angle);
        void Acceleration(const AccType &acc_type);
        void GoStraight();
        void FrontWallCorrection();
        void Back();
        void Wait();

        void SetM_Iden();
        void SetStep_Iden();
        void SetPartyTrick();

        void M_Iden();
        void Step_Iden();
        void PartyTrick();
        void SideWallCorrection();
        void PivotTurnRight90();
        void PivotTurnLeft90();
        void PivotTurn180();
        void CalcSlalomInput();
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
        bool GetCtrlFlag() { return flag_controller; };
        bool GetMazeLoadFlag() { return flag_maze_load; };

        void Reset();
        void ResetWallFlag() { flag_wall = false; };
        void ResetMazeLoadFlag() { flag_maze_load = false; };
        void MotorTest(float v_left, float v_right);
        void Logger();
        void OutputLog();
        void OutputSlalomLog();
        void OutputPivotTurnLog();
        void OutputTranslationLog();
        void OutputMIdenLog() { iden_m.OutputLog(); };
        void OutputStepIdenLog() { iden_step.OutputLog(); };

        bool wallDataReady() { return flag_wall; };
        void updateWallData();
        Direction getWallData();
        void UpdatePos(const Direction &dir);
        void UpdateDir(const Direction &dir);
        IndexVec getRobotPosition() { return robot_position; };
        void robotMove();
        void DirMove(const Direction &dir);
        void DirMoveSlalom(const Direction &dir);
        void OpMove(const Operation &op);
        void OpMoveSlalom(const Operation &op);

    private:
        Speaker *speaker;
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
        hardware::IR_Base *ir_is_wall;
        trajectory::Velocity *velocity;
        undercarriage::Identification iden_m;
        undercarriage::Step_Identification iden_step;

        float v_left;
        float v_right;
        float u_w;
        float u_v;
        int ref_size;
        IR_Value ir_value;
        ctrl::Pose ref_pos{0, 0, 0};  // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0};  // robot coordinates
        ctrl::Pose ref_acc{0, 0, 0};  // robot coordinates
        const float acc_x_err = 30.0; // error threshold
        const float vel_x_err = 20.0; // error threshold
        const float Tp1_w = 31.83;
        const float Kp_w = 144.2;
        const float Tp1_v = 0.032;
        const float Kp_v = 0.784493;
        int vel_mode = 1;
        float ref_v;
        float ref_w;
        const int back_time = 400;       // ms
        const int correction_time = 500; // ms
        const int wait_time = 200;       // ms
        bool flag_controller = false;
        bool flag_slalom;
        bool flag_wall = false; // flag for sensors reading wall
        bool flag_side_wall_left = true;
        bool flag_side_wall_right = true;
        bool flag_front_wall = false;
        bool flag_safety = false;
        bool flag_maze_load = false;
        bool flag_side_correct = true;
        bool flag_straight_wall = false;
        bool flag_straight_time = false;
        int cnt_blind_alley = 0;
        int cnt_time = 0;
        int index_log = 0;
        float theta_base = 0.0;
        float length;
        ctrl::Pose cur_pos{0, 0, 0};
        ctrl::Pose cur_vel{0, 0, 0};
        float acc_x;
        // float base_theta;
        float error_fl;
        float error_fr;
        float *log_x;
        float *log_y;
        float *log_theta;
        float *log_l;
        float *log_v;
        float *log_a;
        float *log_ref_l;
        float *log_ref_v;
        float *log_ref_a;
        float *log_omega;
        float *log_kanayama_v;
        float *log_kanayama_w;
        hardware::IR_Value ir_side_value;
        hardware::IR_Value ir_wall_value;

        int prev_wall_cnt = 0;
        int8_t dir_diff = 0;
        IndexVec robot_position = {0, 0};
        Direction robot_dir = NORTH;
        bool flag_read_side_wall = false;
    };
} // namespace undercarriage

#endif //  CONTROLLER_H_