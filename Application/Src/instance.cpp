/**
 * @file instance.cpp
 * @author Reiji Terunuma
 */

#include "instance.h"

Maze maze;
Maze maze_backup;
Agent agent(maze);
State state;

const float ir_start_base = 2500;

// Tokyo Tech
// const float ir_fl_wall = 2190;
// const float ir_fr_wall = 2190;
// const float ir_sl_wall = 2150;
// const float ir_sr_wall = 2150;
// const float ir_fl_base = 2180;
// const float ir_fr_base = 2180;
// const float ir_sl_base = 3800;
// const float ir_sr_base = 3700;
// const float ir_sl_slalom = 2380;
// const float ir_sr_slalom = 2470;

// Tokyo Polytechnic University
// const float ir_fl_wall = 2180;
// const float ir_fr_wall = 2220;
// const float ir_sl_wall = 2160;
// const float ir_sr_wall = 2160;
// const float ir_fl_base = 2250;
// const float ir_fr_base = 2260;
// const float ir_sl_base = 3330;
// const float ir_sr_base = 3400;
// const float ir_sl_slalom = 2430;
// const float ir_sr_slalom = 2400;

// RT
const float ir_fl_wall = 2220; // WALL_TIMING 0.8
const float ir_fr_wall = 2160;
const float ir_sl_wall = 2150;
const float ir_sr_wall = 2150;
const float ir_fl_base = 2220; // side wall correction
const float ir_fr_base = 2220;
const float ir_sl_base = 3650; // front wall correction
const float ir_sr_base = 3550;
const float ir_slalom = 2420; // front wall correction (slalom)

hardware::IR_Value ir_value;
// for wall judgement
hardware::IR_Base ir_is_wall = {ir_fl_wall, ir_fr_wall, ir_sl_wall, ir_sr_wall};
// for front/side wall correction
hardware::IR_Base ir_ctrl_base = {ir_fl_base, ir_fr_base, ir_sl_base, ir_sr_base, ir_slalom};
hardware::IR_LogCoeff ir_log = {.a = 41.01f, .b = 1.314e-5f, .c = -0.02817f, .d = 262.2f};

hardware::IR_Param ir_param = {ir_is_wall, ir_ctrl_base, ir_log};
hardware::IRsensor irsensors(ir_start_base, &ir_is_wall);

const float control_period = 0.001;
const float sampling_period = 0.001;

undercarriage::Odometory odom(sampling_period);

PID pid_angle(4.0f, 0.0, 0.0, 0.0, control_period);
PID pid_rotational_vel(1.1976f, 85.1838f, -0.00099f, 0.0039227f, control_period);
// PID pid_traslational_vel(0.0068176f, 0.0820249f, -0.000033349f, 0.023191f, control_period);
PID pid_traslational_vel(0.009f, 0.09f, 0.0, 0.0, control_period);
PID pid_ir_front_left(0.0005f, 0.000005f, 0.0, 0.0, control_period);
PID pid_ir_front_right(0.0005f, 0.000005f, 0.0, 0.0, control_period);
PID pid_ir_side(0.003f, 0.000, 0.0, 0.0, control_period);

PID_Instances pid(&pid_angle,
                  &pid_rotational_vel,
                  &pid_traslational_vel,
                  &pid_ir_front_left,
                  &pid_ir_front_right,
                  &pid_ir_side);

undercarriage::Kanayama kanayama(3.0f, 0.003f, 1.0f);
undercarriage::DynamicFeedback dynamic_feedback(10.0f, 0.5f, control_period);
undercarriage::TimeVaryingFeedback time_varying_feedback(1.0f, 1e-3f);
// undercarriage::TrackerBase *tracker = &kanayama;
undercarriage::TrackerBase *tracker = &dynamic_feedback;
// undercarriage::TrackerBase *tracker = &time_varying_feedback;

// translational velocity
trajectory::Velocity velocity = {.v1 = 180.0f, .v2 = 200.0f, .v3 = 300.0f, .v4 = 400.0f, .v5 = 500.0f};

trajectory::Parameter acc_param1 = {.v_max = 3.0e+2f, .a_max = 1.5e+3f, .j_max = 1e+4f};
trajectory::Parameter acc_param2 = {.v_max = 7.0e+3f, .a_max = 1.0e+4f, .j_max = 2e+4f};
trajectory::Parameters acc_params = {.run1 = acc_param1, .run2 = acc_param2, .run3 = acc_param2, .run4 = acc_param2, .run5 = acc_param2};

trajectory::Parameter slalom_param1 = {.v_max = 2.0f * M_PI, .a_max = 10.0f * M_PI, .j_max = 1.0e+3f * M_PI};
trajectory::Parameter slalom_param2 = {.v_max = 5.0f * M_PI, .a_max = 30.0f * M_PI, .j_max = 1.0e+3f * M_PI};
trajectory::Parameters slalom_params = {.run1 = slalom_param1, .run2 = slalom_param2, .run3 = slalom_param2, .run4 = slalom_param2, .run5 = slalom_param2};

trajectory::Slalom slalom(&velocity, &slalom_params);
trajectory::Acceleration acc(&velocity, &acc_params);

undercarriage::Controller controller(&odom,
                                     &pid,
                                     tracker,
                                     &slalom,
                                     &acc,
                                     &ir_param,
                                     &velocity);