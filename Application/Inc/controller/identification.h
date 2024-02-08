/**
 * @file identification.h
 * @date Jan 27th, 2024
 * @brief system identification for undercarriage
 * @author Teruru-52
 */

#ifndef INDENTIFICATION_H_
#define INDENTIFICATION_H_

#include "controller/trajectory.h"
#include "hardware/motor.h"

/**
 * @brief namespace for undercarriage control
 */
namespace undercarriage
{
    /**
     * @brief Base class for identification
     */
    class IdentificationBase
    {
    protected:
        float u;
        bool flag;
        int index;
        int ref_size;
        float *output;

    public:
        explicit IdentificationBase() : flag(false), index(0){};
        virtual float GetInput(const float cur_vel) = 0;
        bool Finished() { return flag; };
        virtual void OutputLog() = 0;
        virtual ~IdentificationBase() { delete[] output; };
    };

    /**
     * @brief class for M sequence identification
     */
    class M_Identification : public IdentificationBase
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

    /**
     * @brief class for step identification
     */
    class Step_Identification : public IdentificationBase
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

#endif //  INDENTIFICATION_H_