#ifndef __MYWINDOW_H__
#define __MYWINDOW_H__

#include "LowerBodyBuilder.h"
#include "Controller.h"
#include "fsm.h"
#include "dart_basic.h"
#include <chrono>
#define EPSILON 0.0001
const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

int tot_dof=9; // root(translational) 2 + revolute joint angle 1(torso)+ 3 * 2(left,right)
int P_NUM=4; //number of goal poses
int cur_p_idx;
int step=0;

VectorXd goalPos(tot_dof);
MatrixXd goalPoses(tot_dof,P_NUM);
VectorXd goalPos_0(tot_dof);
VectorXd goalPos_1(tot_dof);
VectorXd goalPos_2(tot_dof);
VectorXd goalPos_3(tot_dof);



//#include "MyWorld.h"

class MyWindow : public dart::gui::SimWindow
{
    public:
        MyWindow(WorldPtr _mWorld, FSM* fsm);
        void timeStepping() override;
        void keyboard(unsigned char key, int x, int y) override;
        void printKeyboardInstruction();
        
        bool useRender();
        //get Functions
        SkeletonPtr getCharacter();
        SkeletonPtr getGoalVisualize();
        FSM* getFSM();
        Controller* getController();

  private:
        bool render;
        SkeletonPtr mCharacter;
        SkeletonPtr mGoalVisualize;

        FSM* mFSM;
        Controller* mController;
//

};

#endif


