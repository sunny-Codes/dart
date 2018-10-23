#include "MyWorld.h"


/// Constructor
MyWorld::MyWorld(FSM* fsm)//WorldPtr world,  Character *character)
    :hide(false)
{
    render=true;
    //mWindow=dart::gui::SimWindow() ;
    //mWindow.setWorld(world);
    
    mFSM= fsm;
    mCharacter= getSkeleton("character"); //world->getSkeleton("character");
    if(render) mGoalVisualize= getSkeleton("goal"); //world->getSkeleton("goal");
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
}

SkeletonPtr MyWorld::getCharacter()
{
return mCharacter;
}


SkeletonPtr MyWorld::getGoalVisualize()
{
return mGoalVisualize;
}

FSM* MyWorld::getFSM(){
    return mFSM;
}
Controller* MyWorld::getController(){
    return mController;
}



void MyWorld::timeStepping() 
{
    mController->timeStepping(render);

    // Step the simulation forward
    mController->setGoalPos(mFSM->timeStepping()) ; // *mController);
    setPosition(mGoalVisualize, mFSM->get_goalPos());

}



///// functions needed in simulation (main)


/*
void MyWorld::printKeyboardInstruction(){
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
void MyWorld::keyboard(unsigned char key, int x, int y) 
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
            //default:
                //mWindow.keyboard(key, x, y); //Simwindow::
        }
    }
}
*/

bool MyWorld::doRender(){
    return render;
}


