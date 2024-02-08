/**
 * @file main_exec.h
 * @author Teruru-52
 * @brief Main execution process for the robot
 * @date Febrary 13th, 2023
 */

#ifndef MAIN_EXEC_H_
#define MAIN_EXEC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

    void StartupProcess();
    void Initialize();
    void UpdateUndercarriage();
    void UpdateIRSensor();
    void Notification();
    void MazeSearch();
    void TimeAttack();
    void StateProcess();
    void FlashMaze();
    void LoadMaze();

#ifdef __cplusplus
};
#endif

#endif // MAIN_EXEC_H_