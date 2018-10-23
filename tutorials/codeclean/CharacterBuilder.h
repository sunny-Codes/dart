#ifndef __CHARACTER_H__
#define __CHARACTER_H__
#include "BodyNodeBuilder.h"

class CharacterBuilder{
    public:
        CharacterBuilder() {}
        CharacterBuilder(bool _draw, float _stiffness, float _damping, float _rest_position): 
            draw(_draw), stiffness(_stiffness), damping(_damping), rest_position(_rest_position){
                cout<<draw<<endl;
                cout<<stiffness<<endl;
                cout<<damping<<endl;
                cout<<rest_position<<endl;
            }
        //void setBoneGeometry(Vector3d bg){ bone_geometry= bg; }

       //SkeletonPtr getSkeletonPtr(){return skel;}
        //virtual SkeletonPtr buildBody();

    protected:

        //share(maybe)
        bool draw;
        float stiffness;
        float damping;
        float rest_position;

        //Vector3d bone_geometry;
        //SkeletonPtr skel;
};

void changeRestPosition(SkeletonPtr skel, double delta);
void changeStiffness(SkeletonPtr skel, double delta);
void changeDamping(SkeletonPtr skel, double delta);
void setPosition(SkeletonPtr skel, VectorXd def_pos);
void setCollidableFalse(SkeletonPtr skel);


#endif
