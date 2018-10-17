/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 :*   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
#include "Character.h"
#include "LowerBody.h"
#include "Controller.h"
#include "fsm.h"
#include "dart_basic.h"
#include <chrono>
//#include <boost/python.hpp>
//#include <boost/python/list.hpp>
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

class MyWindow : public dart::gui::SimWindow
{
    public:

        /// Constructor
        MyWindow(WorldPtr world, FSM* fsm, Character *character);
        void keyboard(unsigned char key, int x, int y) override;
        void timeStepping() override;
    protected:
        Character *mCharacter;
        SkeletonPtr mCharacterSkelPtr;
        FSM* mFSM;
        Controller * mController;

        bool hide;
}; 
