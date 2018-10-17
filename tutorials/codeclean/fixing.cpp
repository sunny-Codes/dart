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
//#include <boost/python.hpp>
//#include <boost/python/list.hpp>

#include <chrono>

#define EPSILON 0.0001
const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

// desired 
const double desired_height = 1.0; // m
const double desired_width = 0.2;  // m
const double desired_depth = 0.2;  // m
const double desired_rest_position = 0.0;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9*default_ground_width/2;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
using namespace Eigen;
// goal position
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


void setHideOrShow(SkeletonPtr skel, bool hide){
    if(hide){
        for( int i=0; i<skel->getNumShapeNodes(); i++){
            skel->getShapeNode(i)->getVisualAspect()->hide();
        }
    }else{
        for( int i=0; i<skel->getNumShapeNodes(); i++){
            skel->getShapeNode(i)->getVisualAspect()->show();
        }
    }
}
void setColor(BodyNode * bn, Vector3d color){
    for( int i=0; i<bn->getNumShapeNodes(); i++){
        bn->getShapeNode(i)->getVisualAspect()->setColor(color);
    }
}

class MyWindow : public dart::gui::SimWindow
{
    public:

        /// Constructor
        MyWindow(WorldPtr world, FSM* fsm, Character *character)
            : mBallConstraint(nullptr),
            //mPositiveSign(true),
            mBodyForce(false),
            //mPD(false),
            //automode(false),
            hide(false)
    {
        setWorld(world);
        mFSM= fsm;
        mCharacter= character;
        mCharacterSkelPtr = mCharacter->getSkeletonPtr(); //world->getSkeleton("character");
        assert(mCharacterSkelPtr != nullptr);
        mController= new Controller(mCharacterSkelPtr, 300, 30, mFSM);

        mController->mForceCountDown.resize(mCharacterSkelPtr->getNumDofs(), 0);
        ArrowShape::Properties arrow_properties;
        arrow_properties.mRadius = 0.05;
        mController->mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
                    Vector3d(-default_height, 0.0, default_height / 2.0),
                    Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
                    arrow_properties, dart::Color::Orange(1.0)));

        // Set PD control gains
        /*mKpPD = 300.0;
        mKdPD = 30.0;
        */

        //start = std::chrono::system_clock::now();
    }

        /// Add a constraint to attach the final link to the world
        void addConstraint()
        {
            cout<<"addConstraint entered"<<endl;
            // Get the last body in the pendulum
            BodyNode* tip  = mCharacterSkelPtr->getBodyNode(mCharacterSkelPtr->getNumBodyNodes() - 1);

            // Attach the last link to the world
            Vector3d location =
                tip->getTransform() * Vector3d(0.0, 0.0, default_height);
            mBallConstraint =
                std::make_shared<dart::constraint::BallJointConstraint>(tip, location);
            mWorld->getConstraintSolver()->addConstraint(mBallConstraint);
        }

        /// Remove any existing constraint, allowing the pendulum to flail freely
        void removeConstraint()
        {
            cout<<"removeConstraint entered"<<endl;
            mWorld->getConstraintSolver()->removeConstraint(mBallConstraint);
            mBallConstraint = nullptr;
        }
        /// Handle keyboard input
        void keyboard(unsigned char key, int x, int y) override
        {
            switch(key)
            {
                case '-':
                    mController->changeForceDirection();
                    break;

                case '1':
                    mController->applyForce(0);
                    break;
                case '2':
                    mController->applyForce(1);
                    break;
                case '3':
                    mController->applyForce(2);
                    break;
                case '4':
                    mController->applyForce(3);
                    break;
                case '5':
                    mController->applyForce(4);
                    break;
                case '6':
                    mController->applyForce(5);
                    break;
                case '7':
                    mController->applyForce(6);
                    break;
                case '8':
                    mController->applyForce(7);
                    break;
                case '9':
                    mController->applyForce(8);
                    break;
                case '0':
                    mController->applyPDForces(goalPos);
                    break;
                case '/':
                    /*mPD= !mPD;
                    cout<<"mPD now: ";
                    if(mPD) cout<<"true"<<endl;
                    else cout<<"false"<<endl;
                    */
                    mController->changePDmode();
                    cout<<"PD mode now: ";
                    if(mController->usePDcontrol()) cout<<"true"<<endl;
                    else cout<<"false"<<endl;
                    break;
                case 'n':
                    if(!mFSM->isAutomode()){ // go to next state
                        cur_p_idx= (cur_p_idx+1)% P_NUM;
                        goalPos= goalPoses.col(cur_p_idx);
                        cout<<"State: "<<cur_p_idx<<endl;
                    }else{
                        cout<<"'n' pressed, but now in automode"<<endl;
                    }
                    break;
                case 'c':
                    //change the mode
                    mFSM->change_automode();
                    cout<<"automode: ";
                    if(mFSM->isAutomode())cout<<"true"<<endl;
                    else cout<<"false"<<endl;
                    break;
                case 'q':
                    mCharacter->changeRestPosition(delta_rest_position);
                    break;
                case 'a':
                    mCharacter->changeRestPosition(-delta_rest_position);
                    break;

                case 'w':
                    mCharacter->changeStiffness(delta_stiffness);
                    break;
                case 's':
                    mCharacter->changeStiffness(-delta_stiffness);
                    break;

                case 'e':
                    mCharacter->changeDamping(delta_damping);
                    break;
                case 'd':
                    mCharacter->changeDamping(-delta_damping);
                    break;

                case 'r':
                    {
                        if(mBallConstraint)
                            removeConstraint();
                        else
                            addConstraint();
                        break;
                    }

                case 'f':
                    mBodyForce = !mBodyForce;
                    break;

                case 'h':
                    // hide the goal pendulum
                    hide= !hide;
                    setHideOrShow(mWorld->getSkeleton("goal"), hide);
                    break;
                default:
                    SimWindow::keyboard(key, x, y);
            }
        }

        void timeStepping() override
        {
            mController->timeStepping();
            /*for(vector<Controller>::Cit= controller_list.begin(); Cit!= controller_list.end(); Cit++){
              Controller controller= *Cit;
              controller.timeStepping();
              }*/

            /*
               if(render)
               {
               mWindow.timeStepping();
               }*/

            // mWindow.timeStepping is as below
             for(std::size_t i = 0; i < mCharacterSkelPtr->getNumBodyNodes(); ++i)
            {
                BodyNode* bn = mCharacterSkelPtr->getBodyNode(i);
                auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
                //for(std::size_t j = 0; j < 1; ++j)

                visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Green());
                if(visualShapeNodes.size()>1){
                    visualShapeNodes[1]->getVisualAspect()->setColor(dart::Color::Orange());
                }       

                // If we have three visualization shapes, that means the arrow is
                // attached. We should remove it in case this body is no longer
                // experiencing a force
                if(visualShapeNodes.size() == 3u)
                {
                    assert(visualShapeNodes[2]->getShape() == mController->mArrow);
                    visualShapeNodes[2]->remove();
                }
            }
            SimWindow::timeStepping();
            // Step the simulation forward
            mController->setGoalPos(mFSM->timeStepping()) ; // *mController);

            for (int i=0;i<tot_dof;i++){
                //if(i==2) mWorld->getSkeleton("goal_pendulum")->getRootBodyNode()->setProperties
                mWorld->getSkeleton("goal")->setPosition(i, mFSM->get_goalPos()[i]);
            }


        }

    protected:

        /// An arrow shape that we will use to visualize applied forces
        //std::shared_ptr<ArrowShape> mArrow;

        /// The pendulum that we will be perturbing
        Character *mCharacter;
        SkeletonPtr mCharacterSkelPtr;
        FSM* mFSM;
        Controller * mController;


        /// Pointer to the ball constraint that we will be turning on and off
        dart::constraint::BallJointConstraintPtr mBallConstraint;

        /// Number of iterations before clearing a force entry
        //std::vector<int> mForceCountDown;

        /// Whether a force should be applied in the positive or negative direction
        //bool mPositiveSign;

        /// True if 1-9 should be used to apply a body force. Otherwise, 1-9 will be
        /// used to apply a joint torque.
        bool mBodyForce;

        /// Control gains for the proportional error terms in the PD controller
        //double mKpPD;

        /// Control gains for the derivative error terms in the PD controller
        //double mKdPD;

        /// Joint forces for the manipulator (output of the Controller)
        //VectorXd mForces;

        // True if using PD control (change by '0' keyboard)
        //bool mPD;

        //bool automode;
        bool hide;
       // chrono::system_clock::time_point start;
};

void print_bn(BodyNodePtr & bn){
    Isometry3d tf= bn->getWorldTransform();
    Vector4d center= tf.matrix() * Vector4d(0,0,0,1); 
    cout<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
}

void print_Skeleton(SkeletonPtr skel){
    VectorXd pos= skel->getPositions();
    cout<<pos<<endl<<endl;
    int n_bn = skel->getNumBodyNodes();
    for(int i=0; i<n_bn; i++){
        BodyNodePtr bn= skel->getBodyNode(i);
        print_bn(bn);
    }
}

SkeletonPtr createFloor()
{
    SkeletonPtr floor = Skeleton::create("floor");

    // Give the floor a body
    BodyNodePtr body =
        floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 50.0;
    double floor_height = 1;
    std::shared_ptr<BoxShape> box(
            new BoxShape(Vector3d(floor_width, floor_height, floor_width)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Black());

    // Put the body into position
    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = Vector3d(0.0, -2.6, 0.0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}


///// functions needed in simulation (main)



int main(int argc, char* argv[])
    //void simulation()
{

    VectorXd def_pos(9);
    //def_pos<< 0, 0, 180*M_PI/180.0, 0, 0, 90*M_PI/180.0, 0,0, 90*M_PI/180.0;

    def_pos<< 0,0,0,0,0,0,0,0,0;
    //0,2 stance, 1,3 foot strike
    //0: 012 swh swk swa sth* stk sta 
    goalPos_0<< 0, 0, 0, 0.4, -1.1, 0.2, 0, -0.05, 0.2;
    goalPos_1<< 0, 0, 0, -0.7, -0.05, 0.2, 0, -0.1, 0.2;
    goalPos_2<< 0, 0, 0, 0, -0.05, 0.2, 0.4, -1.1, 0.2;
    goalPos_3<< 0, 0, 0, 0, -0.1, 0.2, -0.7, -0.05, 0.2;

    FSM_state s0 (300, 9, goalPos_0);
    FSM_state s1 (30, 9, goalPos_1);
    FSM_state s2 (300, 9, goalPos_2);
    FSM_state s3 (30, 9, goalPos_3);

    FSM fsm=FSM();
    fsm.add_state(s0);
    fsm.add_state(s1);
    fsm.add_state(s2);
    fsm.add_state(s3);
    fsm.add_transition(transition(0, 0, 1));
    fsm.add_transition(transition(1, 0, 2));
    fsm.add_transition(transition(2, 0, 3));
    fsm.add_transition(transition(3, 0, 0));
    fsm.set_start();

    fsm.print_fsm();
    goalPoses << goalPos_0, goalPos_1, goalPos_2, goalPos_3;
    cur_p_idx=1;
    goalPos= goalPoses.col(cur_p_idx); //goalPos_1;

    LowerBody character_lb("character", true, default_stiffness, default_damping, default_rest_position);
    character_lb.setBoneGeometry(Vector3d(default_width, default_height, default_depth));
    SkeletonPtr character= character_lb.buildBody();
    character_lb.setDefault(goalPos_0);

    //'TODO' need to make goal without any skel-> convert it to character if needed
    LowerBody goal_lb("goal", true, default_stiffness, default_damping, default_rest_position);
    goal_lb.setBoneGeometry(Vector3d(default_width, default_height, default_depth));
    SkeletonPtr goal= goal_lb.buildBody();
    goal_lb.setDefault(goalPos);
    for (int i=0;i<goal->getNumBodyNodes();i++){
        goal->getBodyNode(i)->setCollidable(false);
    }
    // Create a world and add the pendulum to the world
    WorldPtr world= std::make_shared<World>();

    world->addSkeleton(character);
    world->addSkeleton(goal);
    world->addSkeleton(createFloor());
    world->setGravity(Vector3d(0,-9.8,0));
    // Create a window for rendering the world and handling user input
    MyWindow window(world, &fsm, &character_lb);

    //print_Skeleton(pendulum);
    // Print instructions
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': replay simulation" << endl;
    cout << "'1' -> '9': apply torque to a pendulum body" << endl;
    cout << "'-': Change sign of applied joint torques" << endl;
    cout << "'q': Increase joint rest positions" << endl;
    cout << "'a': Decrease joint rest positions" << endl;
    cout << "'w': Increase joint spring stiffness" << endl;
    cout << "'s': Decrease joint spring stiffness" << endl;
    cout << "'e': Increase joint damping" << endl;
    cout << "'d': Decrease joint damping" << endl;
    cout << "'r': add/remove constraint on the end of the chain" << endl;
    cout << "'f': switch between applying joint torques and body forces" << endl;
    cout<<"--------- simbicon -----------"<<endl;
    cout << "'0': apply PD Force" << endl;
    cout<<"'/': change PD force mode (continuously apply PD Force or not)"<<endl;
    cout<<"'c': change automode (automode= state changes by time, !automode= state stays there, can change state by 'n')"<<endl;
    cout<<"'n': if NOT automode- go to next state"<<endl;
    // Initialize glut, initialize the window, and begin the glut event loop
    glutInit(&argc, argv);
    window.initWindow(1080, 810, "2D walking body");
    glutMainLoop();
}

