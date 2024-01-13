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
const float ir_fl_wall = 2150;
const float ir_fr_wall = 2150;
const float ir_sl_wall = 2150;
const float ir_sr_wall = 2150;
const float ir_fl_base = 2250;
const float ir_fr_base = 2220;
const float ir_sl_base = 3240;
const float ir_sr_base = 3380;
const float ir_sl_slalom = 2500;
const float ir_sr_slalom = 2500;

hardware::IR_Value ir_value;
// for wall judge
hardware::IR_Base ir_is_wall = {ir_fl_wall, ir_fr_wall, ir_sl_wall, ir_sr_wall};
// for front/side wall correction
hardware::IR_Base ir_base = {ir_fl_base, ir_fr_base, ir_sl_base, ir_sr_base, ir_sl_slalom, ir_sr_slalom};
hardware::IRsensor irsensors(ir_start_base, &ir_is_wall);

const float control_period = 0.001;
const float sampling_period = 0.001;

undercarriage::Odometory odom(sampling_period);

PID pid_angle(4.0, 0.0, 0.0, 0.0, control_period);
PID pid_rotational_vel(1.1976, 85.1838, -0.00099, 0.0039227, control_period);
PID pid_traslational_vel(6.8176, 82.0249, -0.033349, 0.023191, control_period);
// PID pid_traslational_vel(20.0, 100.0, -0.033349, 0.023191, control_period);
PID pid_ir_sensor_front_left(0.0005, 0.000005, 0.0, 0.0, control_period);
PID pid_ir_sensor_front_right(0.0005, 0.000005, 0.0, 0.0, control_period);
// PID pid_ir_sensor_side(0.001, 0.0, 0.00005, 0.001, control_period);
PID pid_ir_sensor_side(0.003, 0.000, 0.0, 0.0, control_period);
undercarriage::Kanayama kanayama(3.0, 3.0, 1.0);
undercarriage::Dynamic_Feedback dynamic_feedback(1.0, 0.05, 1.0, 0.05, control_period);

// const float v1 = 0.186825;
// const float v1 = 0.2594938;
// const float v1 = 0.469949951;
// const float v1 = 0.544497925;
// const float v1 = 0.643138489;
trajectory::Velocity velocity = {.v1 = 0.186825, .v2 = 0.469949951};
// trajectory::Velocity velocity = {.v1 = 0.2594938, .v2 = 0.469949951};

trajectory::Slalom slalom;
trajectory::Acceleration acc(&velocity);

undercarriage::Controller controller(&speaker,
                                     &odom,
                                     &pid_angle,
                                     &pid_rotational_vel,
                                     &pid_traslational_vel,
                                     &pid_ir_sensor_front_left,
                                     &pid_ir_sensor_front_right,
                                     &pid_ir_sensor_side,
                                     &kanayama,
                                     &dynamic_feedback,
                                     &slalom,
                                     &acc,
                                     &ir_base,
                                     &ir_is_wall,
                                     &velocity);