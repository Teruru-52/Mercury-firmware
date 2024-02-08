/**
 * @file instance.h
 * @brief This file is used to declare the instances of the classes
 * @author Teruru-52
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "controller/controller.h"
#include "controller/tracker.h"
#include "machine_state.h"

extern hardware::LED led;
extern hardware::Speaker speaker;
extern hardware::IRsensor irsensors;
extern hardware::IR_Value ir_value;
extern Maze maze;
extern Maze maze_backup;
extern Agent agent;
extern State state;
extern undercarriage::Controller controller;

#endif // INSTANCE_H_