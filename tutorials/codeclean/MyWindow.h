/*
#ifndef __MYWINDOW_H__
#define __MYWINDOW_H__
#include "MyWorld.h"

class MyWindow : public dart::gui::SimWindow
{
    public:
        MyWindow(MyWorld * _mWorld): mWorld(_mWorld){}
        void timeStepping() override;
        void keyboard(unsigned char key, int x, int y) override;
        void printInstruction();
    private:
        World * mWorld;
};

#endif

*/
