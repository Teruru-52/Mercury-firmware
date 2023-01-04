#include "instance.h"

const float control_period = 0.001;
const float sampling_period = 0.001;

undercarriage::Odometory odom(sampling_period);

PID pid_angle(4.0, 0.0, 0.0, 0.0, control_period);
PID pid_rotational_vel(1.1976, 85.1838, -0.00099, 0.0039227, control_period);
PID pid_traslational_vel(6.8176, 82.0249, -0.033349, 0.023191, control_period);
PID pid_ir_sensor_front(0.0005, 0.0005, 0.0, 0.0, control_period);
PID pid_ir_sensor_side(0.001, 0.001, 0.0, 0.0, control_period);
undercarriage::Kanayama kanayama(3.0, 3.0, 10.0);

ctrl::slalom::Shape ss_turn90_1(ctrl::Pose(90, 90, M_PI / 2), 80, 0, 500 * M_PI, 5 * M_PI, M_PI);

const float v1 = 0.186825;

const std::array<float, 8> parameters_stop1 = {10, 1.5, 0.5, v1, 0, 0.09, 0, 0};
const std::array<float, 8> parameters_start1 = {10, 1.5, 0.5, 0, v1, 0.138, 0, 0};

trajectory::Slalom slalom(&ss_turn90_1);
trajectory::Acceleration acc(parameters_start1,
                             parameters_stop1);

undercarriage::Controller controller(&odom,
                                     &pid_angle,
                                     &pid_rotational_vel,
                                     &pid_traslational_vel,
                                     &pid_ir_sensor_front,
                                     &pid_ir_sensor_side,
                                     &kanayama,
                                     &slalom,
                                     &acc);