#include "instance.h"

Maze maze;
Maze maze_backup;
Agent agent(maze);
State state;

hardware::LED led;
hardware::Speaker speaker;

const float ir_start_base = 2500;
const float ir_wall_base = 2150;
const float ir_fl_base = 2220;
const float ir_fr_base = 2280;
const float ir_sl_base = 3670;
const float ir_sr_base = 3540;
hardware::IR_Base ir_base = {ir_wall_base, ir_fl_base, ir_fr_base, ir_sl_base, ir_sr_base};
hardware::IRsensor irsensors(ir_start_base, ir_wall_base);

const float control_period = 0.001;
const float sampling_period = 0.001;

undercarriage::Odometory odom(sampling_period);

PID pid_angle(4.0, 0.0, 0.0, 0.0, control_period);
PID pid_rotational_vel(1.1976, 85.1838, -0.00099, 0.0039227, control_period);
PID pid_traslational_vel(6.8176, 82.0249, -0.033349, 0.023191, control_period);
PID pid_ir_sensor_front_left(0.000005, 0.000005, 0.0, 0.0, control_period);
PID pid_ir_sensor_front_right(0.000005, 0.000005, 0.0, 0.0, control_period);
PID pid_ir_sensor_side(0.001, 0.001, 0.0, 0.0, control_period);
undercarriage::Kanayama kanayama(3.0, 3.0, 10.0);
undercarriage::Dynamic_Feedback dynamic_feedback(1.0, 0.05, 1.0, 0.05, control_period);

ctrl::slalom::Shape ss_turn90_1(ctrl::Pose(90, 90, M_PI / 2), 80, 0, 500 * M_PI, 5 * M_PI, M_PI);

const float parameters_stop1[8] = {10, 1.5, 0.5, 0, 0, 0.09, 0, 0};
const float parameters_start1[8] = {10, 1.5, 0.5, 0, 0, 0.133, 0, 0};
const float parameters_forward1[8] = {10, 1.5, 0.5, 0, 0, 0.18, 0, 0};

const float v1 = 0.186825;
// const float* parameters_stop1[8] = {10, 1.5, 0.5, v1, 0, 0.09, 0, 0};
// const float* parameters_start1[8] = {10, 1.5, 0.5, 0, v1, 0.138, 0, 0};
// const float* parameters_forward1[8] = {10, 1.5, 0.5, v1, v1, 0.18, 0, 0};

trajectory::Slalom slalom(&ss_turn90_1);
trajectory::Acceleration acc(parameters_start1,
                             parameters_forward1,
                             parameters_stop1);

undercarriage::Controller controller(&odom,
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
                                     &ir_base);