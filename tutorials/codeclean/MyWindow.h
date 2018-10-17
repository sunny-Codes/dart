#ifndef __MYWINDOW_H__
#define __MYWINDOW_H__
#include "World.h"

class MyWindow : public dart::gui::SimWindow
{
    public:
        MyWindow(World * _mWorld): mWorld(_mWorld){}
        void timeStepping();
        void keyboard(unsigned char key, int x, int y) override;
        void printInstruction();
    private:
        World * mWorld;
};

#endif
