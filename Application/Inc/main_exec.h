/*
 * main_exec.h
 *
 *  Created on: Febrary 13th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef MAIN_EXEC_H_
#define MAIN_EXEC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

    void StartupProcess();
    void SelectFunc(int16_t pulse_l, int16_t pulse_r);
    void Initialize();
    void UpdateUndercarriage();
    void UpdateIRSensor();
    void Notification();
    void MazeSearch();
    // void TimeAttack();
    void StateProcess();
    void FlashMaze();
    void LoadMaze();

#ifdef __cplusplus
};
#endif

#endif // MAIN_EXEC_H_