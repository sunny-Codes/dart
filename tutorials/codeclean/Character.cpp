#include "Character.h"

void Character::changeRestPosition(double delta)
{
    for(std::size_t i = 0; i < skel->getNumDofs(); ++i)
    {
        DegreeOfFreedom* dof = skel->getDof(i);
        double q0 = dof->getRestPosition() + delta;
        cout<<i<<"\t"<<dof->getRestPosition()<<"\t"<<q0<<endl;
        // The system becomes numerically unstable when the rest position exceeds
        // 90 degrees
        if(std::abs(q0) > 90.0 * M_PI / 180.0)
            q0 = (q0 > 0)? (90.0 * M_PI / 180.0) : -(90.0 * M_PI / 180.0);

        dof->setRestPosition(q0);
    }

    // Only curl up along one axis in the BallJoint
    // skel->getDof(0)->setRestPosition(0.0);
    // skel->getDof(2)->setRestPosition(0.0);
}

void Character::changeStiffness(double delta)
{
    for(std::size_t i = 0; i < skel->getNumDofs(); ++i)
    {
        DegreeOfFreedom* dof = skel->getDof(i);
        double stiffness = dof->getSpringStiffness() + delta;
        if(stiffness < 0.0)
            stiffness = 0.0;
        dof->setSpringStiffness(stiffness);
    }
}

void Character::changeDamping(double delta)
{
    for(std::size_t i = 0; i < skel->getNumDofs(); ++i)
    {
        DegreeOfFreedom* dof = skel->getDof(i);
        double damping = dof->getDampingCoefficient() + delta;
        if(damping < 0.0)
            damping = 0.0;
        dof->setDampingCoefficient(damping);
    }
}


