#include "../Inc/trajectory.h"

namespace trajectory
{
    // Acceleration1
    Acceleration1::Acceleration1()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void Acceleration1::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void Acceleration1::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_v[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    int Acceleration1::GetRefSize()
    {
        return ref_v.size();
    }

    bool Acceleration1::GetFlag()
    {
        return flag;
    }

    void Acceleration1::ResetFlag()
    {
        flag = true;
    }

    float Acceleration1::GetRefVelocity()
    {
        return ref;
    }

    // Acceleration2
    Acceleration2::Acceleration2()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void Acceleration2::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void Acceleration2::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_v[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    int Acceleration2::GetRefSize()
    {
        return ref_v.size();
    }

    bool Acceleration2::GetFlag()
    {
        return flag;
    }

    void Acceleration2::ResetFlag()
    {
        flag = true;
    }

    float Acceleration2::GetRefVelocity()
    {
        return ref;
    }

    // Acceleration3
    Acceleration3::Acceleration3()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void Acceleration3::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void Acceleration3::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_v[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    int Acceleration3::GetRefSize()
    {
        return ref_v.size();
    }

    bool Acceleration3::GetFlag()
    {
        return flag;
    }

    void Acceleration3::ResetFlag()
    {
        flag = true;
    }

    float Acceleration3::GetRefVelocity()
    {
        return ref;
    }

    // Acceleration4
    Acceleration4::Acceleration4()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    void Acceleration4::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void Acceleration4::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_v[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    int Acceleration4::GetRefSize()
    {
        return ref_v.size();
    }

    bool Acceleration4::GetFlag()
    {
        return flag;
    }

    void Acceleration4::ResetFlag()
    {
        flag = true;
    }

    float Acceleration4::GetRefVelocity()
    {
        return ref;
    }

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