#include "World.h"


/// Constructor
MyWindow::MyWindow(WorldPtr world, FSM* fsm, Character *character)
    :hide(false)
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
}
/// Handle keyboard input
void MyWindow::keyboard(unsigned char key, int x, int y) 
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
        case 'f':
            mController->changeBodyForce();
            //mBodyForce = !mBodyForce;
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

void MyWindow::timeStepping() 
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

