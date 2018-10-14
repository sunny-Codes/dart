#ifndef __LOWERBODY_H__
#define __LOWERBODY_H__
#include "BodyNodeBuilder.h"

class LowerBody{
    public:
        LowerBody(string _name): name(_name){}
        LowerBody(string _name, bool _draw, float _stiffness, float _damping, float _rest_position): name(_name), draw(_draw), stiffness(_stiffness), damping(_damping), rest_position(_rest_position){}
        void setBoneGeometry(Vector3d bg){ bone_geometry= bg; }
        /*void set_torso_cst();
        void set_thigh_cst();
        void set_tibia_cst();
        void set_foot_cst();
        */
        SkeletonPtr buildBody();
        void set_default(VectorXd def_pos);

    private:
        BodyNode * addBody_default(
                const SkeletonPtr &skel, BodyNode*parent, const string& name);

        Vector3d V3_Zero=Vector3d(0,0,0);
        string name;
        
        //share(maybe)
        bool draw;
        float stiffness;
        float damping;
        float rest_position;
        
        Vector3d bone_geometry;
        SkeletonPtr skel;
};

#endif
