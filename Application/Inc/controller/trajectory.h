/**
 * @file trajectory.h
 * @brief trajectory generation for undercarriage
 * @author Teruru-52
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "main.h"
#include "slalom.h"
// #include <chrono>

#define ENABLE_SLALOM 0

#define FORWARD_LENGTH 180.0
#define FORWARD_LENGTH_HALF 90.0
#define FORWARD_LENGTH_START 138.0

/**
 * @brief namespace for trajectory generation
 */
namespace trajectory
{
    /**
     * @brief struct for velocity selection
     */
    struct Velocity
    {
        float v1;
        float v2;
        float v3;
        float v4;
        float v5;
    };

    /**
     * @brief struct for trajectory generation parameters
     */
    struct Parameter
    {
        float v_max;
        float a_max;
        float j_max;
    };

    /**
     * @brief struct for runnning parameters
     */
    struct Parameters
    {
        Parameter run1;
        Parameter run2;
        Parameter run3;
        Parameter run4;
        Parameter run5;
    };

    /**
     * @brief base class for online trajectory generation
     */
    class OnlineTrajectoryBase
    {
    protected:
        Velocity *velocity;
        Parameters *params;
        float v_ref = 0;
        Parameter param;

        bool flag_trj = false;
        int trj_mode = 1;
        const float Ts = 1e-3;
        float t = 0;
        float t_end = 0;
        bool flag_read_side_wall = false;
        bool flag_time = false;

    public:
        explicit OnlineTrajectoryBase(Velocity *velocity, Parameters *params) : velocity(velocity), params(params){};
        void SetMode(int trj_mode) { this->trj_mode = trj_mode; };
        int GetRefSize() { return t_end * 1e+3; };
        virtual void UpdateRef() = 0;
        bool Finished() { return flag_trj; };
        void Reset();
        bool GetWallFlag() { return flag_read_side_wall; };
        void ResetWallFlag() { flag_read_side_wall = false; };
        virtual ~OnlineTrajectoryBase() {}
    };

    /**
     * @brief class for trajectory generation for slalom
     */
    class Slalom : public OnlineTrajectoryBase
    {
    public:
        typedef enum
        {
            left_90,
            right_90,
            left_45,
            right_45,
        } SlalomType;

        explicit Slalom(Velocity *velocity, Parameters *params) : OnlineTrajectoryBase(velocity, params),
                                                                  ss_turn90(ctrl::slalom::Shape(ctrl::Pose(90, 90, M_PI / 2), 80, 0, params->run1.j_max, params->run1.a_max, params->run1.v_max)),
                                                                  ss(ss_turn90),
                                                                  st(ctrl::slalom::Trajectory(ss))
        {
            ResetTrajectory();
        };
        void ResetTrajectory(const SlalomType &slalom_type = left_90, float ref_theta = M_PI * 0.5, ctrl::Pose cur_pos = {0, 0, 0});
        void UpdateRef() override;
        ctrl::Pose GetRefPosition() { return ref_pos; };
        ctrl::Pose GetRefVelocity() { return ref_vel; };
        ctrl::Pose GetRefAcceleration() { return ref_acc; };

    private:
        ctrl::slalom::Shape ss_turn90;
        ctrl::slalom::Shape ss;
        ctrl::slalom::Trajectory st;
        ctrl::State state;
        SlalomType slalom_type;

        ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
        ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
    };

    /**
     * @brief class for trajectory generation for straight forward
     */
    class Acceleration : public OnlineTrajectoryBase
    {
    public:
        typedef enum
        {
            start,
            start_half,
            forward,
            stop
        } AccType;

        explicit Acceleration(Velocity *velocity, Parameters *params) : OnlineTrajectoryBase(velocity, params)
        {
            ResetTrajectory();
        };
        void ResetTrajectory(const AccType &acc_type = start, float cur_vel = 0, uint8_t num_square = 1);
        void UpdateRef() override;
        float GetRefPosition() { return ref_pos; };
        float GetRefVelocity() { return ref_vel; };
        float GetRefAcceleration() { return ref_acc; };

    private:
        ctrl::AccelDesigner ad;
        AccType acc_type;

        float ref_pos;
        float ref_vel;
        float ref_acc;
    };

    /**
     * @brief class for offline trajectory generation
     */
    class OfflineTrajectoryBase
    {
    protected:
        int index;
        float ref;
        float ref_a;
        int ref_size;
        bool flag;

    public:
        explicit OfflineTrajectoryBase() : index(0), flag(false) {}
        virtual int GetRefSize() = 0;
        virtual void UpdateRef() = 0;
        bool Finished() { return flag; };
        void Reset();
        float GetRefVelocity() { return ref; };
        float GetRefAcceleration() { return ref_a; };
        virtual ~OfflineTrajectoryBase() {}
    };

    /**
     * @brief class for 90 degree pivot turn
     */
    class PivotTurn90 : public OfflineTrajectoryBase
    {
    public:
        explicit PivotTurn90() { ref_size = GetRefSize(); }
        int GetRefSize() override { return sizeof(ref_w) / sizeof(float); };
        void UpdateRef() override;

    private:
        const float ref_w[313] = {
            0.00000, 0.00175, 0.00700, 0.01575, 0.02800, 0.04375, 0.06300, 0.08575, 0.11200, 0.14175, 0.17500, 0.21175, 0.25200, 0.29575, 0.34300, 0.39375, 0.44800, 0.50575, 0.56700, 0.63175, 0.70000, 0.77175, 0.84700, 0.92575, 1.00800, 1.09375, 1.18300, 1.27575, 1.37200, 1.47175, 1.57500, 1.68000, 1.78500, 1.89000, 1.99500, 2.10000, 2.20500, 2.31000, 2.41500, 2.52000, 2.62500, 2.73000, 2.83500, 2.94000, 3.04500, 3.15000, 3.25500, 3.36000, 3.46500, 3.57000, 3.67500, 3.78000, 3.88500, 3.99000, 4.09500, 4.20000, 4.30500, 4.41000, 4.51500, 4.62000, 4.72500, 4.83000, 4.93500, 5.04000, 5.14500, 5.25000, 5.35500, 5.46000, 5.56500, 5.67000, 5.77500, 5.87825, 5.97800, 6.07425, 6.16700, 6.25625, 6.34200, 6.42425, 6.50300, 6.57825, 6.65000, 6.71825, 6.78300, 6.84425, 6.90200, 6.95625, 7.00700, 7.05425, 7.09800, 7.13825, 7.17500, 7.20825, 7.23800, 7.26425, 7.28700, 7.30625, 7.32200, 7.33425, 7.34300, 7.34825, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.34825, 7.34300, 7.33425, 7.32200, 7.30625, 7.28700, 7.26425, 7.23800, 7.20825, 7.17500, 7.13825, 7.09800, 7.05425, 7.00700, 6.95625, 6.90200, 6.84425, 6.78300, 6.71825, 6.65000, 6.57825, 6.50300, 6.42425, 6.34200, 6.25625, 6.16700, 6.07425, 5.97800, 5.87825, 5.77500, 5.67000, 5.56500, 5.46000, 5.35500, 5.25000, 5.14500, 5.04000, 4.93500, 4.83000, 4.72500, 4.62000, 4.51500, 4.41000, 4.30500, 4.20000, 4.09500, 3.99000, 3.88500, 3.78000, 3.67500, 3.57000, 3.46500, 3.36000, 3.25500, 3.15000, 3.04500, 2.94000, 2.83500, 2.73000, 2.62500, 2.52000, 2.41500, 2.31000, 2.20500, 2.10000, 1.99500, 1.89000, 1.78500, 1.68000, 1.57500, 1.47175, 1.37200, 1.27575, 1.18300, 1.09375, 1.00800, 0.92575, 0.84700, 0.77175, 0.70000, 0.63175, 0.56700, 0.50575, 0.44800, 0.39375, 0.34300, 0.29575, 0.25200, 0.21175, 0.17500, 0.14175, 0.11200, 0.08575, 0.06300, 0.04375, 0.02800, 0.01575, 0.00700, 0.00175};
        const float ref_dw[313] = {
            0.00000, 3.50000, 7.00000, 10.50000, 14.00000, 17.50000, 21.00000, 24.50000, 28.00000, 31.50000, 35.00000, 38.50000, 42.00000, 45.50000, 49.00000, 52.50000, 56.00000, 59.50000, 63.00000, 66.50000, 70.00000, 73.50000, 77.00000, 80.50000, 84.00000, 87.50000, 91.00000, 94.50000, 98.00000, 101.50000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 101.50000, 98.00000, 94.50000, 91.00000, 87.50000, 84.00000, 80.50000, 77.00000, 73.50000, 70.00000, 66.50000, 63.00000, 59.50000, 56.00000, 52.50000, 49.00000, 45.50000, 42.00000, 38.50000, 35.00000, 31.50000, 28.00000, 24.50000, 21.00000, 17.50000, 14.00000, 10.50000, 7.00000, 3.50000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -0.00000, -3.50000, -7.00000, -10.50000, -14.00000, -17.50000, -21.00000, -24.50000, -28.00000, -31.50000, -35.00000, -38.50000, -42.00000, -45.50000, -49.00000, -52.50000, -56.00000, -59.50000, -63.00000, -66.50000, -70.00000, -73.50000, -77.00000, -80.50000, -84.00000, -87.50000, -91.00000, -94.50000, -98.00000, -101.50000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -101.50000, -98.00000, -94.50000, -91.00000, -87.50000, -84.00000, -80.50000, -77.00000, -73.50000, -70.00000, -66.50000, -63.00000, -59.50000, -56.00000, -52.50000, -49.00000, -45.50000, -42.00000, -38.50000, -35.00000, -31.50000, -28.00000, -24.50000, -21.00000, -17.50000, -14.00000, -10.50000, -7.00000, -3.50000};
    };

    /**
     * @brief class for 180 degree pivot turn
     */
    class PivotTurn180 : public OfflineTrajectoryBase
    {
    public:
        explicit PivotTurn180() { ref_size = GetRefSize(); }
        int GetRefSize() override { return sizeof(ref_w) / sizeof(float); };
        void UpdateRef() override;

    private:
        const float ref_w[527] = {
            0.00000, 0.00175, 0.00700, 0.01575, 0.02800, 0.04375, 0.06300, 0.08575, 0.11200, 0.14175, 0.17500, 0.21175, 0.25200, 0.29575, 0.34300, 0.39375, 0.44800, 0.50575, 0.56700, 0.63175, 0.70000, 0.77175, 0.84700, 0.92575, 1.00800, 1.09375, 1.18300, 1.27575, 1.37200, 1.47175, 1.57500, 1.68000, 1.78500, 1.89000, 1.99500, 2.10000, 2.20500, 2.31000, 2.41500, 2.52000, 2.62500, 2.73000, 2.83500, 2.94000, 3.04500, 3.15000, 3.25500, 3.36000, 3.46500, 3.57000, 3.67500, 3.78000, 3.88500, 3.99000, 4.09500, 4.20000, 4.30500, 4.41000, 4.51500, 4.62000, 4.72500, 4.83000, 4.93500, 5.04000, 5.14500, 5.25000, 5.35500, 5.46000, 5.56500, 5.67000, 5.77500, 5.87825, 5.97800, 6.07425, 6.16700, 6.25625, 6.34200, 6.42425, 6.50300, 6.57825, 6.65000, 6.71825, 6.78300, 6.84425, 6.90200, 6.95625, 7.00700, 7.05425, 7.09800, 7.13825, 7.17500, 7.20825, 7.23800, 7.26425, 7.28700, 7.30625, 7.32200, 7.33425, 7.34300, 7.34825, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.35000, 7.34825, 7.34300, 7.33425, 7.32200, 7.30625, 7.28700, 7.26425, 7.23800, 7.20825, 7.17500, 7.13825, 7.09800, 7.05425, 7.00700, 6.95625, 6.90200, 6.84425, 6.78300, 6.71825, 6.65000, 6.57825, 6.50300, 6.42425, 6.34200, 6.25625, 6.16700, 6.07425, 5.97800, 5.87825, 5.77500, 5.67000, 5.56500, 5.46000, 5.35500, 5.25000, 5.14500, 5.04000, 4.93500, 4.83000, 4.72500, 4.62000, 4.51500, 4.41000, 4.30500, 4.20000, 4.09500, 3.99000, 3.88500, 3.78000, 3.67500, 3.57000, 3.46500, 3.36000, 3.25500, 3.15000, 3.04500, 2.94000, 2.83500, 2.73000, 2.62500, 2.52000, 2.41500, 2.31000, 2.20500, 2.10000, 1.99500, 1.89000, 1.78500, 1.68000, 1.57500, 1.47175, 1.37200, 1.27575, 1.18300, 1.09375, 1.00800, 0.92575, 0.84700, 0.77175, 0.70000, 0.63175, 0.56700, 0.50575, 0.44800, 0.39375, 0.34300, 0.29575, 0.25200, 0.21175, 0.17500, 0.14175, 0.11200, 0.08575, 0.06300, 0.04375, 0.02800, 0.01575, 0.00700, 0.00175};
        const float ref_dw[527] = {
            0.00000, 3.50000, 7.00000, 10.50000, 14.00000, 17.50000, 21.00000, 24.50000, 28.00000, 31.50000, 35.00000, 38.50000, 42.00000, 45.50000, 49.00000, 52.50000, 56.00000, 59.50000, 63.00000, 66.50000, 70.00000, 73.50000, 77.00000, 80.50000, 84.00000, 87.50000, 91.00000, 94.50000, 98.00000, 101.50000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 105.00000, 101.50000, 98.00000, 94.50000, 91.00000, 87.50000, 84.00000, 80.50000, 77.00000, 73.50000, 70.00000, 66.50000, 63.00000, 59.50000, 56.00000, 52.50000, 49.00000, 45.50000, 42.00000, 38.50000, 35.00000, 31.50000, 28.00000, 24.50000, 21.00000, 17.50000, 14.00000, 10.50000, 7.00000, 3.50000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -0.00000, -3.50000, -7.00000, -10.50000, -14.00000, -17.50000, -21.00000, -24.50000, -28.00000, -31.50000, -35.00000, -38.50000, -42.00000, -45.50000, -49.00000, -52.50000, -56.00000, -59.50000, -63.00000, -66.50000, -70.00000, -73.50000, -77.00000, -80.50000, -84.00000, -87.50000, -91.00000, -94.50000, -98.00000, -101.50000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -105.00000, -101.50000, -98.00000, -94.50000, -91.00000, -87.50000, -84.00000, -80.50000, -77.00000, -73.50000, -70.00000, -66.50000, -63.00000, -59.50000, -56.00000, -52.50000, -49.00000, -45.50000, -42.00000, -38.50000, -35.00000, -31.50000, -28.00000, -24.50000, -21.00000, -17.50000, -14.00000, -10.50000, -7.00000, -3.50000};
    };

    /**
     * @brief class for M sequence input
     */
    class M_sequence : public OfflineTrajectoryBase
    {
    public:
        explicit M_sequence() { ref_size = GetRefSize(); }
        int GetRefSize() override { return sizeof(ref_u_w) / sizeof(float); };
        void UpdateRef() override;
        float GetRefVoltage() { return ref; };

    private:
        const float ref_u_w[254] =
            {-1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5};
    };
}
#endif //  TRAJECTORY_H_