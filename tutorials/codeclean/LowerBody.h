#ifndef __LOWERBODY_H__
#define __LOWERBODY_H__

#include "Character.h"


class LowerBody: public Character{
    public:
        LowerBody(string _name): Character(_name){}
        LowerBody(string _name, bool _draw, float _stiffness, float _damping, float _rest_position): 
            Character(_name, _draw, _stiffness, _damping, _rest_position){}
        void setBoneGeometry(Vector3d bg){ bone_geometry= bg; }
         void setDefault(VectorXd def_pos);
        /*void set_torso_cst();
        void set_thigh_cst();
        void set_tibia_cst();
        void set_foot_cst();
        */
        SkeletonPtr buildBody();

    private:
        BodyNode * addBody_default(const SkeletonPtr &skel, BodyNode*parent, const string& name);

        Vector3d V3_Zero=Vector3d(0,0,0);
        
        Vector3d bone_geometry;
};

#endif
