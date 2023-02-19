/*
 * main_exec.cpp
 *
 *  Created on: Febrary 13th, 2023
 *      Author: Reiji Terunuma
 */

#include "main_exec.h"
#include "my_header.h"

using AccType = trajectory::Acceleration::AccType;

Direction wallData;
Direction nextDir;
IndexVec robotPos;
Agent::State prevState = Agent::State::IDLE;
OperationList runSequence;

int cnt1kHz = 0;
int cnt1Hz = 0;

float bat_vol;
hardware::IR_Value ir_value;
int16_t pulse;

void StartupProcess()
{
    speaker.Beep();
    irsensors.StartDMA();
    irsensors.BatteryCheck();
    irsensors.on_all_led();
}

void SelectFunc(int16_t pulse)
{
    if (pulse < 8192)
    {
        led.off_back_right();
        led.off_back_left();
        state.func = State::func1;
    }
    else if (pulse < 16384)
    {
        led.on_back_right();
        led.off_back_left();
        state.func = State::func2;
    }
    else if (pulse < 24576)
    {
        led.off_back_right();
        led.on_back_left();
        state.func = State::func3;
    }
    else
    {
        led.on_back_right();
        led.on_back_left();
        state.func = State::func4;
    }
}

void Initialize()
{
    led.off_all();
    speaker.Beep();
    controller.InitializeOdometory();
    speaker.Beep();
    controller.ResetOdometory();

    switch (state.func)
    {
    case State::func1:
        state.mode = State::test;
        break;

    case State::func2:
        controller.SetTrajectoryMode(1);

        // wallData = 0x0E;
        wallData = Direction(14);
        // robotPos = IndexVec(0, 0);
        robotPos = controller.getRobotPosition();
        agent.update(robotPos, wallData);
        // nextDir = NORTH;
        nextDir = Direction(1);

        state.mode = State::search;
        break;

    default:
        break;
    }
    state.interruption = State::interrupt;
}

void UpdateUndercarriage()
{
    bat_vol = irsensors.GetBatteryVoltage();
    controller.UpdateBatteryVoltage(bat_vol);
    irsensors.Update();
    ir_value = irsensors.GetIRSensorData();
    controller.SetIRdata(ir_value);
    controller.UpdateIMU();
    controller.UpdateOdometory();
}

void Notification()
{
    state.interruption = State::not_interrupt;
    speaker.SpeakerOn();
    led.Flashing();
    speaker.SpeakerOff();
    state.interruption = State::interrupt;
}

void MazeSearch()
{
    while (1)
    {
        // while (1)
        // {
        //   if (controller.wallDataReady())
        //   {
        //     controller.ResetWallFlag();
        //     break;
        //   }
        // }
        controller.UpdateDir(nextDir);
        controller.UpdatePos(nextDir);
        wallData = controller.getWallData(ir_value);
        robotPos = controller.getRobotPosition();
        agent.update(robotPos, wallData);
        led.on_back_left();
        if (agent.getState() == Agent::FINISHED)
            break;
        if (prevState == Agent::SEARCHING_NOT_GOAL && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        {
            maze_backup = maze;
            // Notification();
        }
        prevState = agent.getState();
        // if (cnt1Hz > 210 && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        // {
        //   agent.forceGotoStart();
        // }
        nextDir = agent.getNextDirection();
        controller.robotMove2(nextDir);
        led.off_back_left();
        // controller.Wait();
    }
}

// void TimeAttack()
// {
/**********************************
 * 計測走行
 *********************************/
// コマンドリストみたいなやつを取り出す
// runSequence = agent.getRunSequence();
// HAL_Delay(2500);

// // Operationを先頭から順番に実行していく
// for (size_t i = 0; i < runSequence.size(); i++)
// {
//   // Operationの実行が終わるまで待つ(nマス進んだ,右に曲がった)
//   while (!operationFinished())
//     ;

//   // i番目のを実行
//   controller.robotMove(runSequence[i]); // robotMode関数はOperation型を受け取ってそれを実行する関数
// }
// }

void StateProcess()
{
    if (state.mode == State::select_function)
    {
        irsensors.UpdateSideValue();
        pulse = controller.GetPulse();
        SelectFunc(pulse);

        if (irsensors.StartInitialize())
        {
            Initialize();
        }
    }

    else if (state.mode == State::test)
    {
        // for (int i = 0; i < 12; i++)
        // {
        //   controller.PivotTurn(90);
        // }
        // Notification();

        // controller.StartMove();
        // controller.Acceleration(AccType::forward1);
        // controller.Acceleration(AccType::forward1);
        // controller.GoStraight();
        // controller.GoStraight();
        // controller.Turn(90);
        // controller.Turn(-90);
        // controller.Acceleration(AccType::STOP);

        // led.on_back_left();
        state.mode = State::output;
    }

    else if (state.mode == State::output)
    {
        controller.Brake();
        state.interruption = State::not_interrupt;
        irsensors.UpdateSideValue();

        if (Read_GPIO(USER_SW) == 1)
        {
            // controller.OutputLog();
            printf("button OK!\n");
        }
    }

    else if (state.mode == State::search)
    {
        controller.StartMove();
        MazeSearch();
        Notification();

        controller.InitializePosition();
        agent.caclRunSequence(false);

        state.mode = State::select_function;
    }
}
