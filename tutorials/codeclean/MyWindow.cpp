/*
/// Handle keyboard input
void MyWindow::keyboard(unsigned char key, int x, int y) 
{
    mWorld->keyboard(key, x, y);
}

void MyWindow::timeStepping(){
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
*/
