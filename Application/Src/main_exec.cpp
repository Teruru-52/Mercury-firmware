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

float bat_vol;
int16_t pulse_l;
int16_t pulse_r;

int cnt1kHz = 0;
int cnt1Hz = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (state.interruption == State::interrupt)
    {
        if (htim == &htim1) // interruption 84kHz
        {
            irsensors.UpdateSideValue();
            irsensors.UpdateFrontValue();
        }

        if (htim == &htim7) // interruption 1kHz
        {
            cnt1kHz = (cnt1kHz + 1) % 1000;
            UpdateUndercarriage();
            UpdateIRSensor();
            controller.robotMove();

            if (controller.ErrorFlag())
            {
                controller.Brake();
                speaker.SpeakerOn();
                state.mode = State::error;
                state.interruption = State::not_interrupt;
            }

            if (cnt1kHz == 0)
            {
                cnt1Hz++;
                Toggle_GPIO(BACK_RIGHT_LED);
            }

            if (cnt1kHz % 200 == 0)
            {
                if (state.mode == State::test_ir)
                    printf("%lu, %lu,%lu, %lu\n", ir_value.fl, ir_value.fr, ir_value.sl, ir_value.sr);
                else if (state.mode == State::test_odometory)
                    controller.OutputLog();
            }
        }
    }
}

void StartupProcess()
{
    speaker.Beep();
    irsensors.StartDMA();
    irsensors.BatteryCheck();
    HAL_Delay(500);
    irsensors.on_all_led();
}

void Initialize()
{
    led.off_all();
    speaker.Beep();
    controller.InitializeOdometory();
    speaker.Beep();

    wallData = 0x0E;
    robotPos = IndexVec(0, 0);
    agent.update(robotPos, wallData);
    nextDir = NORTH;

    switch (state.func)
    {
    case State::func0: // run slow speed (SEARCHING_NOT_GOAL)
        // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2000);
        controller.SetTrajectoryMode(1);
        state.mode = State::search;
        break;

    case State::func1: // run fast speed (SEARCHING_NOT_GOAL)
        controller.SetTrajectoryMode(2);
        state.mode = State::search;
        break;

    case State::func2: // run slow speed (load maze, SEARCHING_NOT_GOAL)
        LoadMaze();
        controller.SetTrajectoryMode(1);
        state.mode = State::search;
        break;

    case State::func3: // run fast speed (load maze, SEARCHING_NOT_GOAL)
        LoadMaze();
        controller.SetTrajectoryMode(2);
        state.mode = State::search;
        break;

    case State::func4: // run slow speed (load maze, Time Attack)
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2000);
        // LoadMaze();
        // controller.SetTrajectoryMode(1);
        // state.mode = State::run_sequence;
        state.mode = State::output;
        break;

    case State::func5: // run fast speed (load maze, Time Attack)
        LoadMaze();
        controller.SetTrajectoryMode(2);
        state.mode = State::run_sequence;
        break;

    case State::func7:
        state.mode = State::m_identification;
        break;

    case State::func8:
        state.mode = State::step_identification;
        break;

    case State::func9:
        state.mode = State::party_trick;
        break;

    case State::func10:
        controller.SetTrajectoryMode(2);
        state.mode = State::test_slalom2;
        break;

    case State::func11:
        controller.SetTrajectoryMode(1);
        state.mode = State::test_slalom1;
        break;

    case State::func12:
        controller.SetTrajectoryMode(1);
        state.mode = State::test_translation;
        break;

    case State::func13:
        state.mode = State::test_rotation;
        break;

    case State::func14:
        state.mode = State::test_odometory;
        break;

    case State::func15:
        state.mode = State::test_ir;
        break;

    default:
        break;
    }

    switch (state.mazeload)
    {
    case State::load:
        LoadMaze();
        break;

    case State::not_load:
        break;
    }
    state.interruption = State::interrupt;
}

void UpdateUndercarriage()
{
    bat_vol = irsensors.GetBatteryVoltage();
    controller.UpdateBatteryVoltage(bat_vol);
    controller.UpdateOdometory();
}

void UpdateIRSensor()
{
    irsensors.Update();
    ir_value = irsensors.GetIRSensorData();
    controller.SetIRdata(ir_value);
    if (controller.wallDataReady())
    {
        wallData = controller.getWallData();
        controller.ResetWallFlag();
    }
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
        //     if (controller.wallDataReady())
        //     {
        //         controller.ResetWallFlag();
        //         break;
        //     }
        // }
        controller.UpdateDir(nextDir);
        controller.UpdatePos(nextDir);
        robotPos = controller.getRobotPosition();
        agent.update(robotPos, wallData);
        if (agent.getState() == Agent::FINISHED)
        {
            controller.Acceleration(AccType::stop);
            break;
        }
        // if (prevState == Agent::SEARCHING_NOT_GOAL && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        // {
        // maze_backup = maze;
        // }
        prevState = agent.getState();
        if (cnt1Hz > 210 && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        {
            agent.forceGotoStart();
        }
        nextDir = agent.getNextDirection();
        controller.DirMoveSlalom(nextDir); // using slalom
        if (controller.GetMazeLoadFlag())
        {
            FlashMaze();
            controller.ResetMazeLoadFlag();
        }
        // controller.Wait();
    }
}

void TimeAttack()
{
    /**********************************
     * 計測走行
     *********************************/
    // コマンドリストみたいなやつを取り出す
    runSequence = agent.getRunSequence();
    HAL_Delay(2500);

    // // Operationを先頭から順番に実行していく
    controller.Acceleration(AccType::start);
    for (size_t i = 1; i < runSequence.size() - 1; i++)
    {
        // Operationの実行が終わるまで待つ(nマス進んだ,右に曲がった)
        // while (!operationFinished())
        //     ;

        //   // i番目のを実行
        controller.OpMoveSlalom(runSequence[i]); // robotMode関数はOperation型を受け取ってそれを実行する関数
    }
    controller.Acceleration(AccType::stop);
    // state.mode = State::select_function;
    state.mode = State::output;
}

void StateProcess()
{
    if (Read_GPIO(USER_SW) == 0)
        speaker.Beep();

    if (state.mode == State::select_function)
    {
        irsensors.UpdateSideValue();
        pulse_l = controller.GetPulseL();
        pulse_r = controller.GetPulseR();
        state.SelectFunc(pulse_r);
        state.SelectLoadMaze(pulse_l);

        if (irsensors.StartInitialize())
            Initialize();
    }

    else
    {
        switch (state.mode)
        {
        case State::test_slalom1:
            controller.StartMove();
            // controller.Acceleration(AccType::start);
            // controller.GoStraight();
            controller.Turn(-90);
            controller.Acceleration(AccType::stop);
            controller.FrontWallCorrection();
            state.log = State::slalom;
            state.mode = State::output;
            break;

        case State::test_slalom2:
            controller.StartMove();
            // controller.Acceleration(AccType::start);
            controller.GoStraight();
            for (int i = 0; i < 7; i++)
            {
                wallData = controller.getWallData();
                controller.DirMoveSlalom(WEST); // slalom
                controller.Turn(90);
                controller.GoStraight();
                Toggle_GPIO(BACK_LEFT_LED);
            }
            controller.Acceleration(AccType::stop);
            state.log = State::slalom;
            state.mode = State::output;
            break;

        case State::test_rotation:
            for (int i = 0; i < 8; i++)
                controller.PivotTurn(90);
            for (int i = 0; i < 4; i++)
                controller.PivotTurn(180);
            state.log = State::pivot_turn;
            state.mode = State::output;
            break;

        case State::test_translation:
            controller.StartMove();
            for (int i = 0; i < 14; i++)
                controller.GoStraight();
            controller.Acceleration(AccType::stop);
            state.log = State::translation;
            state.mode = State::output;
            break;

        case State::m_identification:
            controller.SetM_Iden();
            state.log = State::m_iden;
            state.mode = State::output;
            break;

        case State::step_identification:
            controller.SetStep_Iden();
            state.log = State::step_iden;
            state.mode = State::output;
            break;

        case State::party_trick:
            controller.SetPartyTrick();
            break;

        case State::output:
            controller.Brake();
            state.interruption = State::not_interrupt;

            if (Read_GPIO(USER_SW) == 0)
            {
                if (state.log == State::slalom)
                    controller.OutputSlalomLog();
                else if (state.log == State::m_iden)
                    controller.OutputMIdenLog();
                else if (state.log == State::step_iden)
                    controller.OutputStepIdenLog();
                else if (state.log == State::pivot_turn)
                    controller.OutputPivotTurnLog();
                else if (state.log == State::translation)
                    controller.OutputTranslationLog();
            }
            break;

        case State::search:
            controller.StartMove();
            MazeSearch();
            controller.InitializePosition();
            Notification();
            // FlashMaze();
            agent.caclRunSequence(false);
            // state.mode = State::select_function;
            state.mode = State::run_sequence;
            break;

        case State::error:
            state.interruption = State::not_interrupt;
            break;

        case State::State::run_sequence:
            TimeAttack();
            break;

        default:
            break;
        }
    }
}

void FlashMaze()
{
    Flash_clear();
    uint32_t *flash_data = (uint32_t *)GetWorkRamPointer();

    for (int y = 0; y < MAZE_SIZE; y++)
    {
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            flash_data[MAZE_SIZE * y + x] = maze.wall[y][x].byte;
        }
    }
    Flash_store();
}

void LoadMaze()
{
    uint32_t *flash_data = (uint32_t *)Flash_load();

    for (int y = 0; y < MAZE_SIZE; y++)
    {
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            maze.wall[y][x].byte = flash_data[MAZE_SIZE * y + x];
        }
    }
}