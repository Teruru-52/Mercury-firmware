/**
 * @file identification.h
 *
 * @date Jan 27th, 2024
 * @author Reiji Terunuma
 */

#ifndef INDENTIFICATION_HPP_
#define INDENTIFICATION_HPP_

#include "main.h"
#include "trajectory.h"
#include "hardware/motor.h"

namespace undercarriage
{
    class Identification
    {
    protected:
        float u;
        bool flag;
        int index;
        int ref_size;
        float *output;

    public:
        explicit Identification() : flag(false), index(0){};
        virtual float GetInput(const float cur_vel) = 0;
        bool Finished() { return flag; };
        virtual void OutputLog() = 0;
        virtual ~Identification() { delete[] output; };
    };

    class M_Identification : public Identification
    {
    private:
        float *input;
        int index_log;
        trajectory::M_sequence m_sequence;

    public:
        explicit M_Identification();
        void UpdateRef();
        float GetInput(const float cur_vel) override;
        void OutputLog() override;
        ~M_Identification() { delete[] input; };
    };

    class Step_Identification : public Identification
    {
    private:
        float input;
        int ref_time = 3000; // [ms]

    public:
        explicit Step_Identification() { output = new float[ref_time]; };
        float GetInput(const float cur_vel) override;
        void OutputLog() override;
    };
} // namespace undercarriage

#endif //  INDENTIFICATION_HPP_