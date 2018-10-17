#include "Controller.h"

void Controller::timeStepping(){
    if(mPDmode){
        applyPDForces(mGoalPos);
    }

    if(!mBodyForce)
    {
        // Apply joint torques based on user input, and color the Joint shape red
        for(std::size_t i = 0; i < skel->getNumDofs(); ++i)
        {
            if(mForceCountDown[i] > 0)
            {
                DegreeOfFreedom* dof = skel->getDof(i);
                dof->setForce( mForceDirection? default_torque : -default_torque );

                /*
                BodyNode* bn = dof->getChildBodyNode();
                auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
                visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());
                */
                --mForceCountDown[i];
            }
        }
    }
    else
    {
        // Apply body forces based on user input, and color the body shape red
        for(std::size_t i = 0; i < skel->getNumBodyNodes(); ++i)
        {
            if(mForceCountDown[i] > 0)
            {
                BodyNode* bn = skel->getBodyNode(i);

                Vector3d force = default_force * Vector3d::UnitX();
                Vector3d location(-default_width / 2.0, 0.0, default_height / 2.0);
                if(!mForceDirection)
                {
                    force = -force;
                    location[0] = -location[0];
                }
                bn->addExtForce(force, location, true, true);

                /*
                auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
                shapeNodes[1]->getVisualAspect()->setColor(dart::Color::Red());
                bn->createShapeNodeWith<VisualAspect>(mArrow);
                */
                --mForceCountDown[i];
            }
        }
    }
    

}
void Controller::applyPDForces(VectorXd goalPos)
{
    if(nullptr == skel)
        return;

    // Compute the joint position error
    VectorXd q = skel->getPositions();
    VectorXd dq = skel->getVelocities();
    q += dq * skel->getTimeStep();

    VectorXd q_err = goalPos - q;

    // Compute the joint velocity error
    VectorXd dq_err = -dq;

    // Compute the joint forces needed to compensate for Coriolis forces and
    // gravity

    //const VectorXd& Cg = skel->getCoriolisAndGravityForces();

    q_err[0]=0;
    q_err[1]=0;
    dq_err[0]=0;
    dq_err[1]=0;

    int cur_state= mFSM->get_cur_state_n();


    bool left_swing = (cur_state==0 || cur_state==1);
    int swinghip= (left_swing)? 3:6;  //3 left, 6 right
    int standhip= (left_swing)? 6:3;

    // Compute the desired joint forces
    //const MatrixXd& M = skel->getMassMatrix();
    mForces = (mKpPD * q_err + mKdPD * dq_err) ; // M*F+ Cg;

    mForces[standhip]= -mForces[2]-mForces[swinghip];


    skel->setForces(mForces);


}

void Controller::changeForceDirection()
{
    mForceDirection = !mForceDirection;
    if(mForceDirection)
    {
        mArrow->setPositions(
                Vector3d(-default_height, 0.0, default_height / 2.0),
                Vector3d(-default_width / 2.0, 0.0, default_height / 2.0));
    }
    else
    {
        mArrow->setPositions(
                Vector3d(default_height, 0.0, default_height / 2.0),
                Vector3d(default_width / 2.0, 0.0, default_height / 2.0));
    }
}

void Controller::applyForce(std::size_t index)
{
    if(index < mForceCountDown.size())
        mForceCountDown[index] = default_countdown;
}

