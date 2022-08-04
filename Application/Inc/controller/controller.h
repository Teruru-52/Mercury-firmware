#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "main.h"
#include <vector>
#include "../hardware/motor.h"
#include "../controller/pid_controller.h"
#include "../controller/kanayama.h"

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(float control_period);

        void UpdateBatteryVoltage(float bat_vol);
        void PartyTrick(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void PivotTurn90(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void PivotTurn180(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void KanayamaTurnLeft90(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void KanayamaTurnRight90(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void GoStraight(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel, const std::vector<uint32_t> &ir_data);
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        void ResetFlag();
        float GetInput();
        void MotorTest(float v_left, float v_right);
        void OutputLog();

    private:
        hardware::Motor motor;
        PID pid_angle;
        PID pid_rotational_vel;
        PID pid_traslational_vel;
        PID pid_ir_sensor_left;
        PID pid_ir_sensor_right;
        undercarriage::Kanayama kanayama;
        trajectory::PivotTurn180 pivot_turn180;
        trajectory::PivotTurn90 pivot_turn90;

        float v_left;
        float v_right;
        float u_w;
        float u_v;
        int ref_size;
        std::vector<float> ref_vel{0, 0};
        const float Tp1_w = 31.83;
        const float Kp_w = 144.2;
        const float Tp1_v = 0.18577;
        const float Kp_v = 0.79586;
        const float ref_v = 0.5064989;
        float ref_w;
        const float ir_straight = 1000;
        bool flag;
        int index_log;
        float *x;
        float *y;
        float *theta;
        float *v;
        float *omega;
        float *kanayama_v;
        float *kanayama_w;
    };
} // namespace undercarriage

#endif //  CONTROLLER_HPP_