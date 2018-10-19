#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "fsm.h"
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
using namespace Eigen;

const double default_torque = 15.0; // N-m
const double default_force =  15.0; // N
const int default_countdown = 200;  // Number of timesteps for applying force

const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = EPSILON;  // m

class Controller{
    public:
        Controller(SkeletonPtr _skel,double _mKpPD, double _mKdPD)
            : skel(_skel), mKpPD(_mKpPD), mKdPD(_mKdPD)  {mPDmode=false; mBodyForce= true; }
        Controller(SkeletonPtr _skel,double _mKpPD, double _mKdPD , FSM* fsm) 
            : skel(_skel), mFSM(fsm) , mKpPD(_mKpPD), mKdPD(_mKdPD)
            {mPDmode=true; mGoalPos=fsm->get_goalPos(); mBodyForce=true; }
        void timeStepping(bool render);
        void changeForceDirection();
        void applyForce(std::size_t index);
        void applyPDForces(VectorXd goalPos);
        void setFSM(FSM* fsm){mFSM=fsm; mPDmode=true; mGoalPos=fsm->get_goalPos();}
        void changePDmode(){mPDmode= !mPDmode; }
        void changeBodyForce(){mBodyForce= !mBodyForce;}
        bool usePDcontrol(){return mPDmode;}
        void setGoalPos(VectorXd newGoalPos){mGoalPos=newGoalPos; }
        vector<int> mForceCountDown;
        std::shared_ptr<ArrowShape> mArrow;


    private:
        SkeletonPtr skel;

        /// mBodyForce: True if 1-9 should be used to apply a body force. Otherwise, 1-9 will be
        /// used to apply a joint torque.
        bool mBodyForce;
        bool mForceDirection; // either positive or negative
        /// Number of iterations before clearing a force entry

        VectorXd mForces;

        // Control gains for the proportional|derivative error terms in the PD controller
        bool mPDmode;
        VectorXd mGoalPos;
        double mKpPD;
        double mKdPD;
        FSM * mFSM;

};

#endif
