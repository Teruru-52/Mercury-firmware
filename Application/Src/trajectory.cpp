#include "../Inc/trajectory.h"

namespace trajectory
{
    // PivotTurn90
    PivotTurn90::PivotTurn90()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void PivotTurn90::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void PivotTurn90::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    int PivotTurn90::GetRefSize()
    {
        return ref_w.size();
    }

    bool PivotTurn90::GetFlag()
    {
        return flag;
    }

    void PivotTurn90::ResetFlag()
    {
        flag = true;
    }

    float PivotTurn90::GetRefVelocity()
    {
        return ref;
    }

    // PivotTurn180
    PivotTurn180::PivotTurn180()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void PivotTurn180::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void PivotTurn180::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    int PivotTurn180::GetRefSize()
    {
        return ref_w.size();
    }

    bool PivotTurn180::GetFlag()
    {
        return flag;
    }

    void PivotTurn180::ResetFlag()
    {
        flag = true;
    }

    float PivotTurn180::GetRefVelocity()
    {
        return ref;
    }

    // TurnLeft
    TurnLeft90::TurnLeft90()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void TurnLeft90::ResetTrajectoryIndex()
    {
        index = 0;
    }

    int TurnLeft90::GetRefSize()
    {
        return ref_csv.size();
    }

    void TurnLeft90::UpdateRef()
    {
        if (index < ref_size - 1)
        {
            ref = ref_csv[index];
            index++;
        }
        else if (index == ref_size - 1)
        {
            flag = false;
        }
    }

    std::vector<float> TurnLeft90::GetRef()
    {
        return ref_csv[index];
    }

    bool TurnLeft90::GetFlag()
    {
        return flag;
    }

    void TurnLeft90::ResetFlag()
    {
        flag = true;
    }

    // M Sequence
    M_sequence::M_sequence()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void M_sequence::ResetTrajectoryIndex()
    {
        index = 0;
    }

    int M_sequence::GetRefSize()
    {
        return ref_u_w.size();
    }

    void M_sequence::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_u_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    float M_sequence::GetRef()
    {
        return ref;
    }

    bool M_sequence::GetFlag()
    {
        return flag;
    }
}