/**
 * @file main_exec.cpp
 * @author Teruru-52
 * @brief Main execution process for the robot
 * @date Febrary 13th, 2023
 */

#include "main_exec.h"
#include "instance.h"
#include "digital_filter.h"
#include "flash.h"

using AccType = trajectory::Acceleration::AccType;

Direction wallData = 0x0E;
Direction nextDir = NORTH;
IndexVec robotPos = IndexVec(0, 0);
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

            // if (controller.GetMazeLoadFlag())
            // {
            //     state.interruption = State::not_interrupt;
            //     FlashMaze();
            //     controller.ResetMazeLoadFlag();
            //     state.interruption = State::interrupt;
            // }

            if (cnt1kHz == 0)
            {
                cnt1Hz++;
                Toggle_GPIO(BACK_RIGHT_LED);
            }

            if (cnt1kHz % 200 == 0)
            {
                if (state.mode == State::test_ir)
                    irsensors.OutputLog();
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
}

void Notification()
{
    // state.interruption = State::not_interrupt;
    speaker.SpeakerOn();
    led.Flashing();
    speaker.SpeakerOff();
    // state.interruption = State::interrupt;
}

void Initialize()
{
    led.off_all();
    switch (state.func)
    {
    case State::func0: // start first searching (not load maze, SEARCHING_NOT_GOAL)
        controller.SetTrajectoryMode(1);
        state.mode = State::search;
        printf("mode: search0\n");
        break;

    case State::func1: // resume searching (load maze, SEARCHING_NOT_GOAL)
        controller.SetTrajectoryMode(1);
        LoadMaze();
        agent.resumeAt(Agent::SEARCHING_NOT_GOAL, maze);
        state.mode = State::search;
        printf("mode: search1\n");
        break;

    case State::func2: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        controller.SetTrajectoryMode(1);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        agent.caclRunSequence(false);
        state.mode = State::run_sequence;
        printf("mode: run_sequence1\n");
        break;

    case State::func3: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        controller.SetTrajectoryMode(2);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        agent.caclRunSequence(false);
        state.mode = State::run_sequence;
        printf("mode: run_sequence2\n");
        break;

    case State::func4: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        controller.SetTrajectoryMode(2);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        agent.caclRunSequence(false);
        state.mode = State::run_sequence;
        printf("mode: run_sequence3\n");
        break;

    case State::func5: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2000);
        controller.SetTrajectoryMode(2);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        agent.caclRunSequence(false);
        state.mode = State::run_sequence;
        printf("mode: run_sequence4\n");
        break;

    case State::func6:
        state.mode = State::m_identification;
        printf("mode: m_identification\n");
        break;

    case State::func7:
        state.mode = State::step_identification;
        printf("mode: step_identification\n");
        break;

    case State::func8:
        state.mode = State::party_trick;
        printf("mode: party_trick\n");
        break;

    case State::func9:
        controller.SetTrajectoryMode(2);
        state.mode = State::test_slalom2;
        printf("mode: test_slalom2\n");
        break;

    case State::func10:
        controller.SetTrajectoryMode(1);
        state.mode = State::test_slalom1;
        printf("mode: test_slalom1\n");
        break;

    case State::func11:
        controller.SetTrajectoryMode(1);
        state.mode = State::test_translation2;
        printf("mode: test_translation2\n");
        break;

    case State::func12:
        controller.SetTrajectoryMode(1);
        state.mode = State::test_translation1;
        printf("mode: test_translation1\n");
        break;

    case State::func13:
        state.mode = State::test_rotation;
        printf("mode: test_rotation\n");
        break;

    case State::func14:
        state.mode = State::test_odometory;
        printf("mode: test_odometory\n");
        break;

    case State::func15:
        // state.mode = State::test_ir;
        state.mode = State::output;
        printf("mode: test_ir\n");
        break;

    default:
        break;
    }

    // switch (state.mazeload)
    // {
    // case State::load:
    //     LoadMaze();
    //     break;

    // case State::not_load:
    //     break;
    // }

    speaker.Beep();
    controller.InitializeOdometory();
    speaker.Beep();
    agent.update(robotPos, wallData);
    state.interruption = State::interrupt;
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
        // printf("pulse_l = %d, pulse_r = %d\n", pulse_l, pulse_r);
        // state.SelectLoadMaze(pulse_l);
        state.SelectFunc(pulse_r);

        if (irsensors.StartInitialize())
            Initialize();
    }

    else
    {
        switch (state.mode)
        {
        case State::search: // func0, func1
            controller.StartMove();
            MazeSearch();
            controller.InitializePosition();
            state.interruption = State::not_interrupt;
            FlashMaze();
            Notification();
            agent.caclRunSequence(false);
            state.interruption = State::interrupt;

            // state.mode = State::select_function;
            state.mode = State::run_sequence;
            // state.log = State::maze;
            // state.mode = State::output;
            break;

        case State::State::run_sequence: // func2, func3, func4, func5
            TimeAttack();
            break;

        case State::m_identification: // func6
            controller.SetM_Iden();
            state.log = State::m_iden;
            state.mode = State::output;
            break;

        case State::step_identification: // func7
            controller.SetStep_Iden();
            state.log = State::step_iden;
            state.mode = State::output;
            break;

        case State::party_trick: // func8
            controller.SetPartyTrick();
            break;

        case State::test_slalom2: // func9
            controller.StartMove();
            // controller.Acceleration(AccType::start);
            controller.GoStraight();
            for (int i = 0; i < 7; i++)
            {
                wallData = controller.getWallData();
                controller.DirMove(WEST); // slalom
                controller.Turn(90);
                controller.GoStraight();
            }
            controller.Acceleration(AccType::stop);
            state.log = State::slalom;
            state.mode = State::output;
            break;

        case State::test_slalom1: // func10
            // controller.StartMove();
            controller.Acceleration(AccType::start);
            controller.GoStraight();
            // controller.Turn(-90);
            controller.Turn(90);
            // controller.Turn(-90);
            // controller.Acceleration(AccType::stop);
            // controller.FrontWallCorrection();
            state.log = State::slalom;
            state.mode = State::output;
            break;

        case State::test_translation2: // func11
            controller.StartMove();
            for (int i = 0; i < 14; i++)
                controller.GoStraight();
            controller.Acceleration(AccType::stop);
            state.log = State::translation;
            state.mode = State::output;
            break;

        case State::test_translation1: // func12
            // controller.StartMove();
            controller.Acceleration(AccType::start_half);
            controller.GoStraight();
            controller.Acceleration(AccType::stop);
            controller.Brake();
            state.log = State::translation;
            state.mode = State::output;
            break;

        case State::test_rotation: // func13
            for (int i = 0; i < 8; i++)
                controller.PivotTurn(90);
            for (int i = 0; i < 4; i++)
                controller.PivotTurn(180);
            state.log = State::pivot_turn;
            state.mode = State::output;
            break;

        case State::test_ir: // func15
            irsensors.UI_led_onoff(ir_value);
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
                else if (state.log == State::maze)
                {
                    for (int y = 0; y < MAZE_SIZE; y++)
                        for (int x = 0; x < MAZE_SIZE; x++)
                            printf("maze.wall[%d][%d].byte = %d\n", y, x, maze.wall[y][x].byte);
                }
            }
            break;

        case State::error:
            state.interruption = State::not_interrupt;
            state.mode = State::output;
            break;

        default:
            break;
        }
    }
}

void MazeSearch()
{
    while (1)
    {
        while (1)
        {
            if (controller.wallDataReady())
            {
                controller.ResetWallFlag();
                break;
            }
        }
        irsensors.UI_led_onoff(controller.GetIRWall());
        // irsensors.PrintWalldata(controller.GetIRWall());
        controller.UpdateDir(nextDir);
        controller.UpdatePos(nextDir);
        wallData = controller.getWallData();
        robotPos = controller.getRobotPosition();
        agent.update(robotPos, wallData);
        if (agent.getState() == Agent::FINISHED)
        {
            controller.Acceleration(AccType::stop);
            break;
        }
        if (agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        {
            // Write_GPIO(BACK_LEFT_LED, GPIO_PIN_SET);
        }
        if (prevState == Agent::SEARCHING_NOT_GOAL && agent.getState() == Agent::BACK_TO_START)
        // if (prevState == Agent::SEARCHING_NOT_GOAL && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        {
            Write_GPIO(BACK_LEFT_LED, GPIO_PIN_SET);
            // maze_backup = maze;
        }
        prevState = agent.getState();
        // if (cnt1Hz > 210 && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        // {
        //     agent.forceGotoStart();
        // }
        nextDir = agent.getNextDirection();
        // printf("nextDir.byte = %d\n", nextDir.byte);
        controller.DirMove(nextDir);
    }
}

void TimeAttack()
{
    /**********************************
     * 計測走行
     *********************************/
    // コマンドリストみたいなやつを取り出す
    runSequence = agent.getRunSequence();

    // // Operationを先頭から順番に実行していく
    controller.Acceleration(AccType::start);
    for (size_t i = 0; i < runSequence.size(); i++)
        controller.OpMove(runSequence[i]);
    controller.Acceleration(AccType::stop);
    state.mode = State::select_function;
    // state.mode = State::output;
}

void FlashMaze()
{
    Flash_clear();
    // uint32_t *flash_data = (uint32_t *)GetWorkRamPointer();
    uint8_t *flash_data = GetWorkRamPointer();

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            flash_data[MAZE_SIZE * y + x] = maze.wall[y][x].byte;

    if (!Flash_store())
        while (1)
            printf("Failed to write flash\n");
}

void LoadMaze()
{
    // uint32_t *flash_data = (uint32_t *)Flash_load();
    uint8_t *flash_data = Flash_load();

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            maze.wall[y][x].byte = flash_data[MAZE_SIZE * y + x];
            printf("maze.wall[%d][%d].byte = %d\n", y, x, maze.wall[y][x].byte);
        }
}