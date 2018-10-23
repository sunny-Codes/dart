#include "MyWindow.h"

/// Handle keyboard input
void MyWindow::keyboard(unsigned char key, int x, int y) 
{
    mWorld->keyboard(key, x, y);
    /*    switch(key)
    {
        case '-':
            mWorld->getController()->changeForceDirection();
            break;

        case '1':
            mWorld->getController()->applyForce(0);
            break;
        case '2':
            mWorld->getController()->applyForce(1);
            break;
        case '3':
            mWorld->getController()->applyForce(2);
            break;
        case '4':
            mWorld->getController()->applyForce(3);
            break;
        case '5':
            mWorld->getController()->applyForce(4);
            break;
        case '6':
            mWorld->getController()->applyForce(5);
            break;
        case '7':
            mWorld->getController()->applyForce(6);
            break;
        case '8':
            mWorld->getController()->applyForce(7);
            break;
        case '9':
            mWorld->getController()->applyForce(8);
            break;
        case '0':
            mWorld->getController()->applyPDForces(goalPos);
            break;
        case '/':
            mWorld->getController()->changePDmode();
            cout<<"PD mode now: ";
            if(mWorld->getController()->usePDcontrol()) cout<<"true"<<endl;
            else cout<<"false"<<endl;
            break;
        case 'n':
            if(!mWorld->getFSM()->isAutomode()){ // go to next state
                cur_p_idx= (cur_p_idx+1)% P_NUM;
                goalPos= goalPoses.col(cur_p_idx);
                cout<<"State: "<<cur_p_idx<<endl;
            }else{
                cout<<"'n' pressed, but now in automode"<<endl;
            }
            break;
        case 'c':
            //change the mode
            mWorld->getFSM()->change_automode();
            cout<<"automode: ";
            if(mWorld->getFSM()->isAutomode())cout<<"true"<<endl;
            else cout<<"false"<<endl;
            break;
        case 'q':
            changeRestPosition(mWorld->getCharacter(), delta_rest_position);
            break;
        case 'a':
            changeRestPosition(mWorld->getCharacter(), -delta_rest_position);
            break;

        case 'w':
            changeStiffness(mWorld->getCharacter(), delta_stiffness);
            break;
        case 's':
            changeStiffness(mWorld->getCharacter(), -delta_stiffness);
            break;

        case 'e':
            changeDamping(mWorld->getCharacter(), delta_damping);
            break;
        case 'd':
            changeDamping(mWorld->getCharacter(), -delta_damping);
            break;
        case 'f':
            mWorld->getController()->changeBodyForce();
            //mBodyForce = !mBodyForce;
            break;
        case 'h':
            // hide the goal pendulum
            //hide= !hide;
            //if(render) setHideOrShow(mGoalVisualize, hide);
            if(render){
                hide= !hide;
                setHideOrShow(mWorld->getGoalVisualize(), hide);
                //mGoalVisualize=nullptr;
                //render=false;
            }
            break;

        default:
            SimWindow::keyboard(key, x, y); //Simwindow::
    }
*/
}

void MyWindow::timeStepping(){

    mWorld->timeStepping();
    
    SkeletonPtr wCharacter= mWorld->getCharacter();
    
    for(std::size_t i = 0; i < wCharacter->getNumBodyNodes(); ++i)
    {
        BodyNode* bn = wCharacter->getBodyNode(i);
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
            assert(visualShapeNodes[2]->getShape() == wCharacter->mArrow);
            visualShapeNodes[2]->remove();
        }
    }

    SimWindow::timeStepping();
 
}

void MyWindow::printKeyboardInstruction(){
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
    cout << "'f': switch between applying joint torques and body forces" << endl;
    cout<<"--------- simbicon -----------"<<endl;
    cout << "'0': apply PD Force" << endl;
    cout<<"'/': change PD force mode (continuously apply PD Force or not)"<<endl;
    cout<<"'c': change automode (automode= state changes by time, !automode= state stays there, can change state by 'n')"<<endl;
    cout<<"'n': if NOT automode- go to next state"<<endl;
 
}

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

    LowerBodyBuilder lbb(true, default_stiffness, default_damping, default_rest_position);
    lbb.setBoneGeometry(Vector3d(default_width, default_height, default_depth));
    SkeletonPtr character= lbb.buildBody("character");
    SkeletonPtr goal= lbb.buildBody("goal");
    setPosition(character, goalPos_0);
    setPosition(goal, goalPos);
    setCollidable(goal);
    
    /*
    //'TODO' need to make goal without any skel-> convert it to character if needed
    LowerBody goal_lb("goal", true, default_stiffness, default_damping, default_rest_position);
    goal_lb.setBoneGeometry(Vector3d(default_width, default_height, default_depth));
    SkeletonPtr goal= goal_lb.buildBody();
    goal_lb.setPosition(goalPos);
    */

    // Create a world and add the pendulum to the world
    World* world= new MyWorld(&fsm); //std::make_shared<MyWorld>(&fsm);

    //MyWorld world= MyWorld(&fsm);

    world->addSkeleton(character);
    world->addSkeleton(goal);
    world->addSkeleton(createFloor());
    world->setGravity(Vector3d(0,-9.8,0));
    // Create a window for rendering the world and handling user input
    //MyWorld dartWorld(world, &fsm); //, &character_lb);

   // Initialize glut, initialize the window, and begin the glut event loop
    MyWindow mWindow(world);
    mWindow.printKeyboardInstruction();
    glutInit(&argc, argv);
    mWindow.initWindow(1080, 810, "2D walking body");
    glutMainLoop();

}


