#ifndef INDENTIFICATION_HPP_
#define INDENTIFICATION_HPP_

#include "main.h"
#include "trajectory.h"
#include "pose.h"

namespace undercarriage
{
    class Identification
    {
    public:
        Identification();
        void UpdateRef();
        float GetRotInput(const ctrl::Pose &cur_vel);
        bool GetFlag() { return flag; };
        void OutputLog();
        trajectory::M_sequence m_sequence;

    private:
        float v_left;
        float v_right;
        float u_w;
        bool flag;
        int index;
        int index_log;
        int ref_size;
        float *input;
        float *output;
    };
} // namespace undercarriage

#endif //  INDENTIFICATION_HPP_