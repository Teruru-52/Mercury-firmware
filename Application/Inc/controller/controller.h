/**
 * @file controller.h
 * @author Reiji Terunuma
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "main.h"
#include "hardware/motor.h"
#include "hardware/speaker.h"
#include "odometory.h"
#include "controller/pid_controller.h"
#include "controller/tracker.h"
#include "trajectory.h"
#include "controller/identification.h"
#include "state.h"
#include "Operation.h"
#include "Maze.h"
#include "Agent.h"

#define ENABLE_LOG 1
#define WALL_TIMING 0.8
#define CNT_BACK 3

using AccType = trajectory::Acceleration::AccType;
using namespace hardware;

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(undercarriage::Odometory *odom,
                   PID_Instances *pid,
                   undercarriage::TrackerBase *tracker,
                   trajectory::Slalom *slalom,
                   trajectory::Acceleration *acc,
                   hardware::IR_Param *ir_param,
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
        } CtrlMode;

        void InitializeOdometory() { odom->Initialize(); };
        int16_t GetPulseL() { return odom->GetPulseL(); };
        int16_t GetPulseR() { return odom->GetPulseR(); };

        void UpdateBatteryVoltage(float bat_vol) { motor.UpdateBatteryVoltage(bat_vol); };
        void UpdateOdometory();
        // void UpdateIMU() { odom->UpdateIMU(); };
        void SetIRdata(const IR_Value &ir_value);
        void SetTrajectoryMode(int trj_mode = 1);
        // void SetBase() { theta_base = cur_pos.th; };
        bool ErrorFlag();

        void SetM_Iden();
        void SetStep_Iden();
        void SetPartyTrick();
        void PivotTurn(int angle);
        void Turn(int angle);
        void Acceleration(const AccType &acc_type, uint8_t num_square = 1);
        void GoStraight();
        void FrontWallCorrection();
        void Back();
        void Wait_ms();

        void M_Iden();
        void Step_Iden();
        void PartyTrick();
        void SideWallCorrection();
        void PivotTurn();
        void CalcSlalomInput();
        float GetFrontWallPos(float ir_fmean);
        void Turn();
        void Acceleration();
        void GoStraight(float ref_l);
        void Back(int time);
        void Wait_ms(int time);
        void FrontWallCorrection(const IR_Value &ir_value);
        void BlindAlley();
        void StartMove();
        void InitializePosition();
        void Brake();
        void InputVelocity(float input_v, float input_w);

        void ResetCtrl();
        bool GetCtrlFlag() { return flag_ctrl; };
        bool GetMazeLoadFlag() { return flag_maze_load; };
        void ResetWallFlag() { flag_wall = false; };
        void ResetMazeLoadFlag() { flag_maze_load = false; };

        bool wallDataReady() { return flag_wall; };
        void updateWallData() { ir_wall_value = ir_value; };
        IR_Value GetIRWall() { return ir_wall_value; };
        Direction getWallData();
        void UpdatePos(const Direction &dir);
        void UpdateDir(const Direction &dir) { robot_dir = dir; };
        IndexVec getRobotPosition() { return robot_position; };

        void robotMove();
        void DirMove(const Direction &dir);
        void OpMove(const Operation &op);

        void Logger();
        void OutputLog();
        void OutputSlalomLog();
        void OutputPivotTurnLog();
        void OutputTranslationLog();
        void OutputMIdenLog() { iden_m.OutputLog(); };
        void OutputStepIdenLog() { iden_step.OutputLog(); };
        void MotorTest(float v_left, float v_right);

    private:
        undercarriage::Odometory *odom;
        hardware::Motor motor;
        PID_Instances *pid;
        undercarriage::TrackerBase *tracker;
        trajectory::Slalom *slalom;
        trajectory::Acceleration *acc;
        trajectory::PivotTurn180 pivot_turn180;
        trajectory::PivotTurn90 pivot_turn90;
        CtrlMode mode_ctrl;
        hardware::IR_Param *ir_param;
        trajectory::Velocity *velocity;
        undercarriage::M_Identification iden_m;
        undercarriage::Step_Identification iden_step;

        float v_left;
        float v_right;
        float u_w;
        float u_v;

        float ref_v;
        float ref_w;
        float ref_dw;
        int ref_size;

        float theta_base = 0.0; // theta_global_ref
        float theta_global = 0.0;
        float theta_error = 0.0;
        int angle_turn;
        float ref_theta = 0;
        float length;
        ctrl::Pose cur_pos{0, 0, 0};
        ctrl::Pose cur_vel{0, 0, 0};
        ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
        ctrl::Pose ref_vel_ctrl{0, 0, 0};
        ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
        float acc_x;
        const float acc_x_err = 30.0 * 1e+3; // error threshold

        const float Tp1_w = 31.83;
        const float Kp_w = 144.2 * 1e+3;
        const float Tp1_v = 0.032;
        // const float Kp_v = 0.784493 * 1e+3;
        const float Kp_v = 0.65 * 1e+3;

        hardware::IR_Value ir_value;
        hardware::IR_Value ir_wall_value;
        bool flag_read_side_wall = false;
        float error_fl;
        float error_fr;

        int prev_wall_cnt = 0;
        int8_t dir_diff = 0;
        IndexVec robot_position = {0, 0};
        Direction robot_dir = NORTH;

        const int back_time = 400;       // ms
        const int correction_time = 500; // ms
        const int wait_time = 200;       // ms
        int cnt_blind_alley = 0;
        int cnt_can_back = 0;
        int cnt_time = 0;

        bool flag_ctrl = false;
        bool flag_slalom;
        bool flag_wall = false; // flag for sensors reading wall
        bool flag_wall_sl = true;
        bool flag_wall_sr = true;
        bool flag_wall_front = false;
        bool flag_safety = false;
        bool flag_maze_load = false;
        bool flag_side_correct = false;
        bool flag_straight_wall = false;
        bool flag_straight_time = false;

        int index_log = 0;
        float *log_x;
        float *log_y;
        // float *log_l;
        float *log_theta;
        float *log_omega;
        float *log_v;
        float *log_a;
        float *log_ref_x;
        float *log_ref_y;
        float *log_ref_theta;
        float *log_ref_omega;
        float *log_ref_v;
        float *log_ref_a;
        float *log_ctrl_v;
        float *log_ctrl_w;
        float *log_u_v;
        float *log_u_w;
    };
} // namespace undercarriage

#endif //  CONTROLLER_H_