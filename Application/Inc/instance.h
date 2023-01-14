#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "hardware/led.h"
#include "hardware/speaker.h"
#include "hardware/ir_sensor.h"
#include "controller/controller.h"
#include "controller/kanayama.h"
#include "controller/dynamic_feedback.h"
#include "controller/pid_controller.h"
#include "trajectory.h"
#include "machine_state.h"

extern hardware::LED led;
extern hardware::Speaker speaker;
extern hardware::IRsensor irsensors;
extern Maze maze;
extern Maze maze_backup;
extern Agent agent;
extern State state;
extern undercarriage::Controller controller;

#endif // INSTANCE_H_