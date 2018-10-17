#ifndef __CHARACTER_H__
#define __CHARACTER_H__
#include "BodyNodeBuilder.h"

class Character{
    public:
        Character(string _name): name(_name){}
        Character(string _name, bool _draw, float _stiffness, float _damping, float _rest_position): 
            name(_name), draw(_draw), stiffness(_stiffness), damping(_damping), rest_position(_rest_position){
                cout<<name<<endl;
                cout<<draw<<endl;
                cout<<stiffness<<endl;
                cout<<damping<<endl;
                cout<<rest_position<<endl;
            }
        //void setBoneGeometry(Vector3d bg){ bone_geometry= bg; }

       void changeRestPosition(double delta);
        void changeStiffness(double delta);
        void changeDamping(double delta);
        SkeletonPtr getSkeletonPtr(){return skel;}
        //virtual SkeletonPtr buildBody();

    protected:
        string name;

        //share(maybe)
        bool draw;
        float stiffness;
        float damping;
        float rest_position;

        //Vector3d bone_geometry;
        SkeletonPtr skel;
};

#endif
