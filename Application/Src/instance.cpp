#include "instance.h"

hardware::LED led;
hardware::Speaker speaker;

Maze maze;
Maze maze_backup;
Agent agent(maze);
State state(&led, &speaker);

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
// for wall judge
hardware::IR_Base ir_is_wall = {ir_fl_wall, ir_fr_wall, ir_sl_wall, ir_sr_wall};
// for front/side wall correction
hardware::IR_Base ir_base = {ir_fl_base, ir_fr_base, ir_sl_base, ir_sr_base, ir_slalom};
hardware::IRsensor irsensors(ir_start_base, &ir_is_wall);

const float control_period = 0.001;
const float sampling_period = 0.001;

undercarriage::Odometory odom(sampling_period);

PID pid_angle(4.0, 0.0, 0.0, 0.0, control_period);
PID pid_rotational_vel(1.1976, 85.1838, -0.00099, 0.0039227, control_period);
// PID pid_traslational_vel(0.0068176, 0.0820249, -0.000033349, 0.023191, control_period);
PID pid_traslational_vel(0.009, 0.09, 0.0, 0.0, control_period);
PID pid_ir_sensor_front_left(0.0005, 0.000005, 0.0, 0.0, control_period);
PID pid_ir_sensor_front_right(0.0005, 0.000005, 0.0, 0.0, control_period);
PID pid_ir_sensor_side(0.003, 0.000, 0.0, 0.0, control_period);

undercarriage::Kanayama kanayama(3.0, 0.003, 1.0);
undercarriage::DynamicFeedback dynamic_feedback(10.0f, 0.5f, control_period);
undercarriage::TimeVaryingFeedback time_varying_feedback(1.0f, 1e-3f);
// undercarriage::TrackerBase *tracker = &kanayama;
// undercarriage::TrackerBase *tracker = &dynamic_feedback;
undercarriage::TrackerBase *tracker = &time_varying_feedback;

hardware::IR_FrontParam ir_fparam = {.a = 41.01, .b = 1.314e-5, .c = -0.02817, .d = 262.2};
// translational velocity
trajectory::Velocity velocity = {.v1 = 180.0, .v2 = 200.0, .v3 = 300.0};

trajectory::Slalom slalom(&velocity);
trajectory::Acceleration acc(&velocity);

undercarriage::Controller controller(&speaker,
                                     &odom,
                                     &pid_angle,
                                     &pid_rotational_vel,
                                     &pid_traslational_vel,
                                     &pid_ir_sensor_front_left,
                                     &pid_ir_sensor_front_right,
                                     &pid_ir_sensor_side,
                                     tracker,
                                     &slalom,
                                     &acc,
                                     &ir_base,
                                     &ir_is_wall,
                                     &ir_fparam,
                                     &velocity);