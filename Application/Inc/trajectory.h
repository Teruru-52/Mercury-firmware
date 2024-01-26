#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "main.h"
#include "slalom.h"
#include "pose.h"
#include "state.h"
#include "accel_designer.h"
#include <chrono>

#define ENABLE_SLALOM 1

#define FORWARD_LENGTH 180.0
#define FORWARD_LENGTH_HALF 90.0
#define FORWARD_LENGTH_START 138.0

namespace trajectory
{
    struct Velocity
    {
        float v1;
        float v2;
        float v3;
        float v4;
    };

    class Slalom
    {
    public:
        Slalom(Velocity *velocity) : velocity(velocity) { ResetTrajectory(); };
        void ResetTrajectory(int angle = 90, float ref_theta = M_PI * 0.5, ctrl::Pose cur_pos = {0, 0, 0});
        void SetMode(int slalom_mode) { this->slalom_mode = slalom_mode; };
        int GetRefSize();
        void UpdateRef();
        ctrl::Pose GetRefPosition() { return ref_pos; };
        ctrl::Pose GetRefVelocity() { return ref_vel; };
        ctrl::Pose GetRefAcceleration() { return ref_acc; };
        bool Finished() { return flag_slalom; };
        void Reset();
        bool GetWallFlag() { return flag_read_side_wall; };
        void ResetWallFlag() { flag_read_side_wall = false; };

    private:
        Velocity *velocity;

        // ctrl::slalom::Shape ss_turn90_1 = ctrl::slalom::Shape(ctrl::Pose(90, 90, M_PI / 2), 80, 0, 500 * M_PI, 5 * M_PI, M_PI);
        ctrl::slalom::Shape ss_turn90_1 = ctrl::slalom::Shape(ctrl::Pose(90, 90, M_PI / 2), 90, 0, 1000 * M_PI, 10 * M_PI, 2.0 * M_PI);
        ctrl::slalom::Shape ss_turn90_2 = ctrl::slalom::Shape(ctrl::Pose(90, 90, M_PI / 2), 90, 0, 1000 * M_PI, 30 * M_PI, 5.0 * M_PI);
        // ctrl::slalom::Shape ss_turn90_1(ctrl::Pose(90, 90, M_PI / 2), 80, 0, 1000 * M_PI, 60 * M_PI, 10.0 * M_PI);
        // ctrl::slalom::Shape ss_turn90_1(ctrl::Pose(90, 90, M_PI / 2), 80, 0, 2000 * M_PI, 200 * M_PI, 20.0 * M_PI);
        ctrl::slalom::Shape ss = ss_turn90_1;                       // initial shape
        ctrl::slalom::Trajectory st = ctrl::slalom::Trajectory(ss); // initial trajectory
        ctrl::State state;

        ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
        ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
        float angle_turn = 0;
        bool flag_slalom = false;
        int slalom_mode = 1;
        const float Ts = 1e-3;
        float v_ref = 0;
        float t = 0;
        float t_end;
        int ref_size;
        bool flag_read_side_wall = false;
        bool flag_time = false;
    };

    class Acceleration
    {
    public:
        typedef enum
        {
            start,
            start_half,
            forward_half,
            forward0,
            forward1,
            stop
        } AccType;

        Acceleration(Velocity *velocity);
        void ResetAccCurve(const AccType &acc_type);
        void SetMode(int acc_mode = 1) { this->acc_mode = acc_mode; };
        int GetRefSize();
        void UpdateRef();
        float GetRefPosition() { return ref_pos; };
        float GetRefVelocity() { return ref_vel; };
        float GetRefAcceleration() { return ref_acc; };
        bool Finished() { return flag_acc; };
        bool GetReadSideWallFlag() { return flag_read_side_wall; };
        void Reset();
        bool GetWallFlag() { return flag_read_side_wall; };
        void ResetWallFlag() { flag_read_side_wall = false; };

    private:
        ctrl::AccelDesigner ad;

        Velocity *velocity;
        // ctrl::AD_Parameters param_stop0;
        // ctrl::AD_Parameters param_start0;
        // ctrl::AD_Parameters param_forward0;
        ctrl::AD_Parameters param_stop1;
        ctrl::AD_Parameters param_start1;
        ctrl::AD_Parameters param_start_half1;
        // ctrl::AD_Parameters param_forward1;
        ctrl::AD_Parameters param_stop2;
        ctrl::AD_Parameters param_start2;
        ctrl::AD_Parameters param_start_half2;
        // ctrl::AD_Parameters param_forward2;
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
        bool flag_read_side_wall = false;
        bool flag_time = false;
    };

    class StaticTrajectoryBase
    {
    protected:
        int index;
        float ref;
        float ref_a;
        int ref_size;
        bool flag;

    public:
        explicit StaticTrajectoryBase() : index(0), flag(false) {}
        virtual int GetRefSize() = 0;
        virtual void UpdateRef() = 0;
        bool Finished() { return flag; };
        void Reset();
        float GetRefVelocity() { return ref; };
        float GetRefAcceleration() { return ref_a; };
        virtual ~StaticTrajectoryBase() {}
    };

    class PivotTurn90 : public StaticTrajectoryBase
    {
    public:
        explicit PivotTurn90() { ref_size = GetRefSize(); }
        int GetRefSize() override { return sizeof(ref_w) / sizeof(float); };
        void UpdateRef() override;

    private:
        const float ref_w[265] = {
            0.00000, 0.00200, 0.00800, 0.01800, 0.03200, 0.05000, 0.07200, 0.09800, 0.12800, 0.16200, 0.20000, 0.24200, 0.28800, 0.33800, 0.39200, 0.45000, 0.51200, 0.57800, 0.64800, 0.72200, 0.80000, 0.88200, 0.96800, 1.05800, 1.15200, 1.25000, 1.35200, 1.45800, 1.56800, 1.68200, 1.80000, 1.92000, 2.04000, 2.16000, 2.28000, 2.40000, 2.52000, 2.64000, 2.76000, 2.88000, 3.00000, 3.12000, 3.24000, 3.36000, 3.48000, 3.60000, 3.72000, 3.84000, 3.96000, 4.08000, 4.20000, 4.32000, 4.44000, 4.56000, 4.68000, 4.80000, 4.92000, 5.04000, 5.16000, 5.28000, 5.40000, 5.52000, 5.64000, 5.76000, 5.88000, 6.00000, 6.12000, 6.24000, 6.36000, 6.48000, 6.60000, 6.72000, 6.84000, 6.96000, 7.08000, 7.20000, 7.32000, 7.44000, 7.56000, 7.68000, 7.80000, 7.92000, 8.04000, 8.16000, 8.28000, 8.40000, 8.52000, 8.64000, 8.76000, 8.88000, 9.00000, 9.11800, 9.23200, 9.34200, 9.44800, 9.55000, 9.64800, 9.74200, 9.83200, 9.91800, 10.00000, 10.07800, 10.15200, 10.22200, 10.28800, 10.35000, 10.40800, 10.46200, 10.51200, 10.55800, 10.60000, 10.63800, 10.67200, 10.70200, 10.72800, 10.75000, 10.76800, 10.78200, 10.79200, 10.79800, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.80000, 10.79800, 10.79200, 10.78200, 10.76800, 10.75000, 10.72800, 10.70200, 10.67200, 10.63800, 10.60000, 10.55800, 10.51200, 10.46200, 10.40800, 10.35000, 10.28800, 10.22200, 10.15200, 10.07800, 10.00000, 9.91800, 9.83200, 9.74200, 9.64800, 9.55000, 9.44800, 9.34200, 9.23200, 9.11800, 9.00000, 8.88000, 8.76000, 8.64000, 8.52000, 8.40000, 8.28000, 8.16000, 8.04000, 7.92000, 7.80000, 7.68000, 7.56000, 7.44000, 7.32000, 7.20000, 7.08000, 6.96000, 6.84000, 6.72000, 6.60000, 6.48000, 6.36000, 6.24000, 6.12000, 6.00000, 5.88000, 5.76000, 5.64000, 5.52000, 5.40000, 5.28000, 5.16000, 5.04000, 4.92000, 4.80000, 4.68000, 4.56000, 4.44000, 4.32000, 4.20000, 4.08000, 3.96000, 3.84000, 3.72000, 3.60000, 3.48000, 3.36000, 3.24000, 3.12000, 3.00000, 2.88000, 2.76000, 2.64000, 2.52000, 2.40000, 2.28000, 2.16000, 2.04000, 1.92000, 1.80000, 1.68200, 1.56800, 1.45800, 1.35200, 1.25000, 1.15200, 1.05800, 0.96800, 0.88200, 0.80000, 0.72200, 0.64800, 0.57800, 0.51200, 0.45000, 0.39200, 0.33800, 0.28800, 0.24200, 0.20000, 0.16200, 0.12800, 0.09800, 0.07200, 0.05000, 0.03200, 0.01800, 0.00800, 0.00200};
        const float ref_dw[265] = {
            0.00000, 4.00000, 8.00000, 12.00000, 16.00000, 20.00000, 24.00000, 28.00000, 32.00000, 36.00000, 40.00000, 44.00000, 48.00000, 52.00000, 56.00000, 60.00000, 64.00000, 68.00000, 72.00000, 76.00000, 80.00000, 84.00000, 88.00000, 92.00000, 96.00000, 100.00000, 104.00000, 108.00000, 112.00000, 116.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 120.00000, 116.00000, 112.00000, 108.00000, 104.00000, 100.00000, 96.00000, 92.00000, 88.00000, 84.00000, 80.00000, 76.00000, 72.00000, 68.00000, 64.00000, 60.00000, 56.00000, 52.00000, 48.00000, 44.00000, 40.00000, 36.00000, 32.00000, 28.00000, 24.00000, 20.00000, 16.00000, 12.00000, 8.00000, 4.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -0.00000, -4.00000, -8.00000, -12.00000, -16.00000, -20.00000, -24.00000, -28.00000, -32.00000, -36.00000, -40.00000, -44.00000, -48.00000, -52.00000, -56.00000, -60.00000, -64.00000, -68.00000, -72.00000, -76.00000, -80.00000, -84.00000, -88.00000, -92.00000, -96.00000, -100.00000, -104.00000, -108.00000, -112.00000, -116.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -120.00000, -116.00000, -112.00000, -108.00000, -104.00000, -100.00000, -96.00000, -92.00000, -88.00000, -84.00000, -80.00000, -76.00000, -72.00000, -68.00000, -64.00000, -60.00000, -56.00000, -52.00000, -48.00000, -44.00000, -40.00000, -36.00000, -32.00000, -28.00000, -24.00000, -20.00000, -16.00000, -12.00000, -8.00000, -4.00000};
    };

    class PivotTurn180 : public StaticTrajectoryBase
    {
    public:
        explicit PivotTurn180() { ref_size = GetRefSize(); }
        int GetRefSize() override { return sizeof(ref_w) / sizeof(float); };
        void UpdateRef() override;

    private:
        const float ref_w[364] = {
            0.00000, 0.00175, 0.00700, 0.01575, 0.02800, 0.04375, 0.06300, 0.08575, 0.11200, 0.14175, 0.17500, 0.21175, 0.25200, 0.29575, 0.34300, 0.39375, 0.44800, 0.50575, 0.56700, 0.63175, 0.70000, 0.77175, 0.84700, 0.92575, 1.00800, 1.09375, 1.18300, 1.27575, 1.37200, 1.47175, 1.57500, 1.68175, 1.79200, 1.90575, 2.02300, 2.14375, 2.26800, 2.39575, 2.52700, 2.66175, 2.80000, 2.94000, 3.08000, 3.22000, 3.36000, 3.50000, 3.64000, 3.78000, 3.92000, 4.06000, 4.20000, 4.34000, 4.48000, 4.62000, 4.76000, 4.90000, 5.04000, 5.18000, 5.32000, 5.46000, 5.60000, 5.74000, 5.88000, 6.02000, 6.16000, 6.30000, 6.44000, 6.58000, 6.72000, 6.86000, 7.00000, 7.14000, 7.28000, 7.42000, 7.56000, 7.70000, 7.84000, 7.98000, 8.12000, 8.26000, 8.40000, 8.54000, 8.68000, 8.82000, 8.96000, 9.10000, 9.24000, 9.38000, 9.52000, 9.66000, 9.80000, 9.94000, 10.08000, 10.22000, 10.36000, 10.50000, 10.64000, 10.78000, 10.92000, 11.06000, 11.20000, 11.33825, 11.47300, 11.60425, 11.73200, 11.85625, 11.97700, 12.09425, 12.20800, 12.31825, 12.42500, 12.52825, 12.62800, 12.72425, 12.81700, 12.90625, 12.99200, 13.07425, 13.15300, 13.22825, 13.30000, 13.36825, 13.43300, 13.49425, 13.55200, 13.60625, 13.65700, 13.70425, 13.74800, 13.78825, 13.82500, 13.85825, 13.88800, 13.91425, 13.93700, 13.95625, 13.97200, 13.98425, 13.99300, 13.99825, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 14.00000, 13.99825, 13.99300, 13.98425, 13.97200, 13.95625, 13.93700, 13.91425, 13.88800, 13.85825, 13.82500, 13.78825, 13.74800, 13.70425, 13.65700, 13.60625, 13.55200, 13.49425, 13.43300, 13.36825, 13.30000, 13.22825, 13.15300, 13.07425, 12.99200, 12.90625, 12.81700, 12.72425, 12.62800, 12.52825, 12.42500, 12.31825, 12.20800, 12.09425, 11.97700, 11.85625, 11.73200, 11.60425, 11.47300, 11.33825, 11.20000, 11.06000, 10.92000, 10.78000, 10.64000, 10.50000, 10.36000, 10.22000, 10.08000, 9.94000, 9.80000, 9.66000, 9.52000, 9.38000, 9.24000, 9.10000, 8.96000, 8.82000, 8.68000, 8.54000, 8.40000, 8.26000, 8.12000, 7.98000, 7.84000, 7.70000, 7.56000, 7.42000, 7.28000, 7.14000, 7.00000, 6.86000, 6.72000, 6.58000, 6.44000, 6.30000, 6.16000, 6.02000, 5.88000, 5.74000, 5.60000, 5.46000, 5.32000, 5.18000, 5.04000, 4.90000, 4.76000, 4.62000, 4.48000, 4.34000, 4.20000, 4.06000, 3.92000, 3.78000, 3.64000, 3.50000, 3.36000, 3.22000, 3.08000, 2.94000, 2.80000, 2.66175, 2.52700, 2.39575, 2.26800, 2.14375, 2.02300, 1.90575, 1.79200, 1.68175, 1.57500, 1.47175, 1.37200, 1.27575, 1.18300, 1.09375, 1.00800, 0.92575, 0.84700, 0.77175, 0.70000, 0.63175, 0.56700, 0.50575, 0.44800, 0.39375, 0.34300, 0.29575, 0.25200, 0.21175, 0.17500, 0.14175, 0.11200, 0.08575, 0.06300, 0.04375, 0.02800, 0.01575, 0.00700, 0.00175};
        const float ref_dw[364] = {
            0.00000, 3.50000, 7.00000, 10.50000, 14.00000, 17.50000, 21.00000, 24.50000, 28.00000, 31.50000, 35.00000, 38.50000, 42.00000, 45.50000, 49.00000, 52.50000, 56.00000, 59.50000, 63.00000, 66.50000, 70.00000, 73.50000, 77.00000, 80.50000, 84.00000, 87.50000, 91.00000, 94.50000, 98.00000, 101.50000, 105.00000, 108.50000, 112.00000, 115.50000, 119.00000, 122.50000, 126.00000, 129.50000, 133.00000, 136.50000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 140.00000, 136.50000, 133.00000, 129.50000, 126.00000, 122.50000, 119.00000, 115.50000, 112.00000, 108.50000, 105.00000, 101.50000, 98.00000, 94.50000, 91.00000, 87.50000, 84.00000, 80.50000, 77.00000, 73.50000, 70.00000, 66.50000, 63.00000, 59.50000, 56.00000, 52.50000, 49.00000, 45.50000, 42.00000, 38.50000, 35.00000, 31.50000, 28.00000, 24.50000, 21.00000, 17.50000, 14.00000, 10.50000, 7.00000, 3.50000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -0.00000, -3.50000, -7.00000, -10.50000, -14.00000, -17.50000, -21.00000, -24.50000, -28.00000, -31.50000, -35.00000, -38.50000, -42.00000, -45.50000, -49.00000, -52.50000, -56.00000, -59.50000, -63.00000, -66.50000, -70.00000, -73.50000, -77.00000, -80.50000, -84.00000, -87.50000, -91.00000, -94.50000, -98.00000, -101.50000, -105.00000, -108.50000, -112.00000, -115.50000, -119.00000, -122.50000, -126.00000, -129.50000, -133.00000, -136.50000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -140.00000, -136.50000, -133.00000, -129.50000, -126.00000, -122.50000, -119.00000, -115.50000, -112.00000, -108.50000, -105.00000, -101.50000, -98.00000, -94.50000, -91.00000, -87.50000, -84.00000, -80.50000, -77.00000, -73.50000, -70.00000, -66.50000, -63.00000, -59.50000, -56.00000, -52.50000, -49.00000, -45.50000, -42.00000, -38.50000, -35.00000, -31.50000, -28.00000, -24.50000, -21.00000, -17.50000, -14.00000, -10.50000, -7.00000, -3.50000};
    };

    class M_sequence : public StaticTrajectoryBase
    {
    public:
        explicit M_sequence() { ref_size = GetRefSize(); }
        void ResetTrajectoryIndex() { index = 0; };
        int GetRefSize() override { return sizeof(ref_u_w) / sizeof(float); };
        void UpdateRef() override;
        float GetRefVoltage() { return ref; };

    private:
        const float ref_u_w[254] =
            {-1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, -1.5, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, 1.5, 1.5, -1.5, -1.5, 1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 1.5, 1.5, 1.5, 1.5};
    };
}
#endif //  TRAJECTORY_H_