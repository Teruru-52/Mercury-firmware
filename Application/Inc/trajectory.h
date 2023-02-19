#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "main.h"
#include "slalom.h"
#include "pose.h"
#include "state.h"
#include "accel_designer.h"
#include <chrono>

namespace trajectory
{
    class Slalom
    {
    public:
        Slalom(ctrl::slalom::Shape *ss_turn90_1);
        void ResetTrajectory(int angle = 90);
        void SetMode(int slalom_mode);
        int GetRefSize();
        void UpdateRef();
        ctrl::Pose GetRefPosition();
        ctrl::Pose GetRefVelocity();
        ctrl::Pose GetRefAcceleration();
        bool Finished();
        void Reset();

    private:
        ctrl::slalom::Shape *ss_turn90_1;
        // ctrl::slalom::Shape ss_turn90_2;
        // ctrl::slalom::Shape ss_turn90_3;
        ctrl::slalom::Shape *ss;
        ctrl::slalom::Trajectory st;
        ctrl::State state;

        ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
        ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
        bool flag_slalom;
        int slalom_mode = 1;
        const float Ts = 1e-3;
        float v = 0;
        float t = 0;
        float t_end;
        int ref_size;
    };

    class Acceleration
    {
    public:
        typedef enum
        {
            start,
            forward_half,
            forward1,
            forward2,
            forward3,
            stop
        } AccType;

        Acceleration(const float *parameters_start1,
                     const float *parameters_forward1,
                     const float *parameters_stop1);
        void ResetAccCurve(const AccType &acc_type);
        void SetMode(int acc_mode = 1);
        int GetRefSize();
        void UpdateRef();
        float GetRefPosition();
        float GetRefVelocity();
        float GetRefAcceleration();
        bool Finished();
        void Reset();

    private:
        ctrl::AccelDesigner ad;
        ctrl::AccelDesigner ad1;
        ctrl::AccelDesigner ad2;
        ctrl::AccelDesigner ad3;

        const float *parameters_start1;
        const float *parameters_forward1;
        const float *parameters_forward_half;
        const float *parameters_stop1;
        AccType acc_type;

        float ref_pos;
        float ref_vel;
        float ref_acc;
        bool flag_acc;
        int acc_mode = 1;
        const float Ts = 1e-3;
        float v = 0;
        float t = 0;
        float t_end;
        int ref_size;
    };

    class PivotTurn90
    {
    public:
        PivotTurn90();
        void UpdateRef();
        int GetRefSize();
        bool Finished();
        void Reset();
        float GetRefVelocity();

    private:
        int index;
        float ref;
        int ref_size;
        bool flag;
        const float ref_w[413] = {
            0,
            0.00175000000000000,
            0.00700000000000000,
            0.0157500000000000,
            0.0280000000000000,
            0.0437500000000000,
            0.0630000000000000,
            0.0857500000000000,
            0.112000000000000,
            0.141750000000000,
            0.175000000000000,
            0.211750000000000,
            0.252000000000000,
            0.295750000000000,
            0.343000000000000,
            0.393750000000000,
            0.448000000000000,
            0.505750000000001,
            0.567000000000001,
            0.631750000000001,
            0.700000000000001,
            0.771750000000001,
            0.847000000000001,
            0.92575000000000,
            1.00800000000000,
            1.09375000000000,
            1.18300000000000,
            1.27575000000000,
            1.37200000000000,
            1.47175000000000,
            1.57500000000000,
            1.68000000000000,
            1.78500000000000,
            1.89000000000000,
            1.99500000000000,
            2.10000000000000,
            2.20500000000000,
            2.31000000000000,
            2.41500000000000,
            2.52000000000000,
            2.62500000000000,
            2.73000000000000,
            2.83500000000000,
            2.94000000000000,
            3.04500000000000,
            3.15000000000000,
            3.25500000000000,
            3.36000000000000,
            3.46500000000000,
            3.57000000000000,
            3.67500000000000,
            3.78000000000000,
            3.88500000000000,
            3.99000000000000,
            4.09500000000000,
            4.20000000000001,
            4.30500000000000,
            4.41000000000001,
            4.51500000000001,
            4.62000000000000,
            4.72500000000001,
            4.83000000000001,
            4.93500000000001,
            5.04000000000000,
            5.14500000000001,
            5.25000000000000,
            5.35500000000001,
            5.46000000000001,
            5.56500000000001,
            5.67000000000001,
            5.77500000000001,
            5.87825000000001,
            5.97800000000001,
            6.07425000000001,
            6.16700000000001,
            6.25625000000001,
            6.34200000000001,
            6.42425000000001,
            6.50300000000001,
            6.57825000000001,
            6.65000000000001,
            6.71825000000001,
            6.78300000000001,
            6.84425000000001,
            6.90200000000001,
            6.95625000000000,
            7.00700000000000,
            7.05425000000000,
            7.09800000000000,
            7.13825000000000,
            7.17500000000000,
            7.20825000000000,
            7.23800000000000,
            7.26425000000000,
            7.28700000000000,
            7.30625000000000,
            7.32200000000000,
            7.33425000000000,
            7.34300000000000,
            7.34825000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.35000000000000,
            7.34825000000000,
            7.35000000000000,
            7.33425000000000,
            7.34300000000000,
            7.30625000000000,
            7.32200000000000,
            7.26425000000000,
            7.28700000000000,
            7.20825000000000,
            7.23800000000000,
            7.13825000000000,
            7.17500000000000,
            7.05424999999999,
            7.09799999999999,
            6.95624999999999,
            7.00699999999999,
            6.84424999999999,
            6.90199999999999,
            6.71824999999999,
            6.78299999999999,
            6.57824999999999,
            6.64999999999999,
            6.42424999999999,
            6.50299999999999,
            6.25624999999999,
            6.34199999999999,
            6.07424999999998,
            6.16699999999998,
            5.87824999999998,
            5.97799999999998,
            5.66999999999998,
            5.77499999999998,
            5.45999999999998,
            5.56499999999998,
            5.24999999999998,
            5.35499999999998,
            5.03999999999998,
            5.14499999999998,
            4.82999999999998,
            4.93499999999998,
            4.61999999999998,
            4.72499999999998,
            4.40999999999998,
            4.51499999999998,
            4.19999999999998,
            4.30499999999998,
            3.98999999999998,
            4.09499999999998,
            3.77999999999998,
            3.88499999999998,
            3.56999999999998,
            3.67499999999998,
            3.35999999999998,
            3.46499999999998,
            3.14999999999998,
            3.25499999999998,
            2.93999999999998,
            3.04499999999998,
            2.72999999999998,
            2.83499999999998,
            2.51999999999998,
            2.62499999999998,
            2.30999999999998,
            2.41499999999998,
            2.09999999999998,
            2.20499999999998,
            1.88999999999998,
            1.99499999999998,
            1.67999999999998,
            1.78499999999998,
            1.47174999999998,
            1.57499999999998,
            1.27574999999998,
            1.37199999999998,
            1.09374999999998,
            1.18299999999998,
            0.925749999999984,
            1.00799999999998,
            0.771749999999985,
            0.846999999999984,
            0.631749999999986,
            0.699999999999986,
            0.505749999999988,
            0.566999999999987,
            0.393749999999989,
            0.447999999999988,
            0.295749999999990,
            0.342999999999990,
            0.211749999999992,
            0.251999999999991,
            0.141749999999993,
            0.174999999999993,
            0.0857499999999947,
            0.111999999999994,
            0.0437499999999962,
            0.0629999999999955,
            0.0157499999999977,
            0.0279999999999969,
            0.00174999999999923,
            0.00699999999999846};
    };

    class PivotTurn180
    {
    public:
        PivotTurn180();
        void UpdateRef();
        int GetRefSize();
        bool Finished();
        void Reset();
        float GetRefVelocity();

    private:
        int index;
        float ref;
        int ref_size;
        bool flag;
        const float ref_w[527] =
            {0,
             0.00175000000000000,
             0.00700000000000000,
             0.0157500000000000,
             0.0280000000000000,
             0.0437500000000000,
             0.0630000000000000,
             0.0857500000000000,
             0.112000000000000,
             0.141750000000000,
             0.175000000000000,
             0.211750000000000,
             0.252000000000000,
             0.295750000000000,
             0.343000000000000,
             0.393750000000000,
             0.448000000000000,
             0.505750000000000,
             0.567000000000000,
             0.631750000000000,
             0.700000000000000,
             0.771750000000000,
             0.847000000000000,
             0.925750000000000,
             1.00800000000000,
             1.09375000000000,
             1.18300000000000,
             1.27575000000000,
             1.37200000000000,
             1.47175000000000,
             1.57500000000000,
             1.68000000000000,
             1.78500000000000,
             1.89000000000000,
             1.99500000000000,
             2.10000000000000,
             2.20500000000000,
             2.31000000000000,
             2.41500000000000,
             2.52000000000000,
             2.62500000000000,
             2.73000000000000,
             2.83500000000000,
             2.94000000000000,
             3.04500000000000,
             3.15000000000000,
             3.25500000000000,
             3.36000000000000,
             3.46500000000000,
             3.57000000000000,
             3.67500000000000,
             3.78000000000000,
             3.88500000000000,
             3.99000000000000,
             4.09500000000000,
             4.20000000000000,
             4.30500000000000,
             4.41000000000000,
             4.51500000000000,
             4.62000000000000,
             4.72500000000000,
             4.83000000000000,
             4.93500000000000,
             5.04000000000000,
             5.14500000000000,
             5.25000000000000,
             5.35500000000000,
             5.46000000000000,
             5.56500000000000,
             5.67000000000000,
             5.77500000000000,
             5.87825000000000,
             5.97800000000000,
             6.07425000000000,
             6.16700000000000,
             6.25625000000000,
             6.34200000000000,
             6.42425000000000,
             6.50300000000000,
             6.57825000000000,
             6.65000000000000,
             6.71825000000000,
             6.78300000000000,
             6.84425000000000,
             6.90200000000000,
             6.95625000000000,
             7.00700000000000,
             7.05425000000000,
             7.09800000000000,
             7.13825000000000,
             7.17500000000000,
             7.20825000000000,
             7.23800000000000,
             7.26425000000000,
             7.28700000000000,
             7.30625000000000,
             7.32200000000000,
             7.33425000000000,
             7.34300000000000,
             7.34825000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.35000000000000,
             7.34825000000000,
             7.34300000000000,
             7.33425000000000,
             7.32200000000000,
             7.30625000000000,
             7.28700000000000,
             7.26425000000000,
             7.23800000000000,
             7.20825000000000,
             7.17500000000000,
             7.13825000000000,
             7.09800000000000,
             7.05425000000000,
             7.00700000000000,
             6.95625000000000,
             6.90200000000000,
             6.84425000000000,
             6.78300000000000,
             6.71825000000000,
             6.65000000000000,
             6.57825000000000,
             6.50300000000000,
             6.42425000000000,
             6.34200000000000,
             6.25625000000000,
             6.16700000000000,
             6.07425000000000,
             5.97800000000000,
             5.87825000000000,
             5.77500000000000,
             5.67000000000000,
             5.56500000000000,
             5.46000000000000,
             5.35500000000000,
             5.25000000000000,
             5.14500000000000,
             5.04000000000000,
             4.93500000000000,
             4.83000000000000,
             4.72500000000000,
             4.62000000000000,
             4.51500000000000,
             4.41000000000000,
             4.30500000000000,
             4.20000000000000,
             4.09500000000000,
             3.99000000000000,
             3.88500000000000,
             3.78000000000000,
             3.67500000000000,
             3.57000000000000,
             3.46500000000000,
             3.36000000000000,
             3.25500000000000,
             3.15000000000000,
             3.04500000000000,
             2.94000000000000,
             2.83500000000000,
             2.73000000000000,
             2.62500000000000,
             2.52000000000000,
             2.41500000000000,
             2.31000000000000,
             2.20500000000000,
             2.10000000000000,
             1.99500000000000,
             1.89000000000000,
             1.78500000000000,
             1.68000000000000,
             1.57500000000000,
             1.47175000000000,
             1.37200000000000,
             1.27575000000000,
             1.18300000000000,
             1.09375000000000,
             1.00800000000000,
             0.925750000000001,
             0.847000000000000,
             0.771750000000001,
             0.700000000000000,
             0.631750000000000,
             0.567000000000000,
             0.505750000000000,
             0.448000000000000,
             0.393750000000000,
             0.343000000000000,
             0.295750000000000,
             0.252000000000001,
             0.211750000000000,
             0.175000000000001,
             0.141750000000000,
             0.112000000000000,
             0.0857500000000000,
             0.0629999999999997,
             0.0437500000000002,
             0.0280000000000005,
             0.0157499999999997,
             0.00699999999999967,
             0.00175000000000036};
    };

    class M_sequence
    {
    public:
        M_sequence();
        void ResetTrajectoryIndex();
        int GetRefSize();
        void UpdateRef();
        float GetRef();
        bool GetFlag();

    private:
        int index;
        float ref;
        bool flag;
        int ref_size;

        const float ref_u_w[254] =
            {-1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             -1.5,
             -1.5,
             -1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             1.5,
             -1.5,
             1.5,
             1.5,
             1.5,
             1.5};
    };
}
#endif //  TRAJECTORY_H_