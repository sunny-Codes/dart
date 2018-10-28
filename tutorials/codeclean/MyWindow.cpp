#include "MyWindow.h"
MyWindow::MyWindow(WorldPtr _mWorld, FSM* fsm){
    render=true;
    //mWorld= _mWorld;
    //mWindow=dart::gui::SimWindow() ;
    setWorld(_mWorld);
    
    mFSM= fsm;
    mCharacter= mWorld->getSkeleton("character"); //world->getSkeleton("character");
    if(render) mGoalVisualize= mWorld->getSkeleton("goal"); //world->getSkeleton("goal");
    //mCharacter = mCharacter->getSkeletonPtr(); //world->getSkeleton("character");
    assert(mCharacter != nullptr);
    mController= new Controller(mCharacter, 300, 30, mFSM);

    mController->mForceCountDown.resize(mCharacter->getNumDofs(), 0);
    ArrowShape::Properties arrow_properties;
    arrow_properties.mRadius = 0.05;
    mController->mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
                Vector3d(-default_height, 0.0, default_height / 2.0),
                Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
                arrow_properties, dart::Color::Orange(1.0)));
    cout<<"render: "<<render<<endl;
}

/// Handle keyboard input
void MyWindow::keyboard(unsigned char key, int x, int y) 
{
    if(render){   
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
                changeRestPosition(mCharacter, delta_rest_position);
                break;
            case 'a':
                changeRestPosition(mCharacter, -delta_rest_position);
                break;

            case 'w':
                changeStiffness(mCharacter, delta_stiffness);
                break;
            case 's':
                changeStiffness(mCharacter, -delta_stiffness);
                break;

            case 'e':
                changeDamping(mCharacter, delta_damping);
                break;
            case 'd':
                changeDamping(mCharacter, -delta_damping);
                break;
            case 'f':
                mController->changeBodyForce();
                //mBodyForce = !mBodyForce;
                break;
                /*
                   case 'h':
                // hide the goal pendulum
                //hide= !hide;
                //if(render) setHideOrShow(mGoalVisualize, hide);
                if(render){
                hide= !hide;
                setHideOrShow(mGoalVisualize, hide);
                //mGoalVisualize=nullptr;
                //render=false;
                }
                break;
                */
            default:
                SimWindow::keyboard(key, x, y); //Simwindow::
        }

    }
}


bool MyWindow::useRender(){
    return render;
}
SkeletonPtr MyWindow::getCharacter()
{
    return mCharacter;
}

SkeletonPtr MyWindow::getGoalVisualize()
{
    return mGoalVisualize;
}

FSM* MyWindow::getFSM(){
    return mFSM;
}
Controller* MyWindow::getController(){
    return mController;
}


void MyWindow::timeStepping(){

    mController->timeStepping(render);
    
    if (render){

        for(std::size_t i = 0; i < mCharacter->getNumBodyNodes(); ++i)
        {
            BodyNode* bn = mCharacter->getBodyNode(i);
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
                assert(visualShapeNodes[2]->getShape() == mCharacter->mArrow);
                visualShapeNodes[2]->remove();
            }
        }

        SimWindow::timeStepping();
        
        setPosition(mGoalVisualize, mFSM->get_goalPos());
    }
    mController->setGoalPos(mFSM->timeStepping()) ; // *mController);

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
    setCollidableFalse(goal);
    
    /*
    //'TODO' need to make goal without any skel-> convert it to character if needed
    LowerBody goal_lb("goal", true, default_stiffness, default_damping, default_rest_position);
    goal_lb.setBoneGeometry(Vector3d(default_width, default_height, default_depth));
    SkeletonPtr goal= goal_lb.buildBody();
    goal_lb.setPosition(goalPos);
    */

    // Create a world and add the pendulum to the world
    WorldPtr world= std::make_shared<World>();

    world->addSkeleton(character);
    world->addSkeleton(goal);
    world->addSkeleton(createFloor());
    world->setGravity(Vector3d(0,-9.8,0));

    MyWindow window(world, &fsm);
    // Initialize glut, initialize the window, and begin the glut event loop
    window.printKeyboardInstruction();
    glutInit(&argc, argv);
    window.initWindow(1080, 810, "2D walking body");
    glutMainLoop();

}


