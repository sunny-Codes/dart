#include "LowerBody.h"
void LowerBody::setDefault(VectorXd def_pos){
    for (int i=0; i<skel->getNumDofs(); i++){
        skel->setPosition(i, def_pos[i]);
    }
}

BodyNode* LowerBody::addBody_default (
        const SkeletonPtr& skel,
        BodyNode* parent, 
        const string& name){
    return addBody(skel, parent, name, 
            Vector3d(0, -bone_geometry[1], 0), V3_Zero, 
            draw, bone_geometry, 
            rest_position, stiffness, damping);
}

SkeletonPtr LowerBody::buildBody(){
    skel = Skeleton::create(name);

    // Add each body to the last BodyNode in the skel
    BodyNode* root = makeTranslational2DRootBody(skel, "root", draw, bone_geometry[0], bone_geometry[2], Vector2d(0,0), stiffness, damping);
    BodyNode *bn_t= addBody(skel, root, "torso", V3_Zero, Vector3d(0,0,M_PI), draw, bone_geometry, rest_position, stiffness, damping);

    double default_width= bone_geometry[0];
    double default_height= bone_geometry[1];
    double default_depth= bone_geometry[2];
 
    BodyNode * bn_l = addBody(skel, root, "leg_l1", V3_Zero, V3_Zero, draw, bone_geometry, rest_position, stiffness, damping);
    bn_l = addBody_default(skel, bn_l, "leg_l2");
    bn_l = addBody(skel, bn_l, "leg_l3", 
            Vector3d(0, -default_height, 0), Vector3d(0,0,M_PI/2.0), 
            draw, Vector3d(default_width, default_height/4.0, default_depth),
            rest_position, stiffness, damping);
 
    BodyNode * bn_r = addBody(skel, root, "leg_r1", V3_Zero, V3_Zero, draw, bone_geometry, rest_position, stiffness, damping);
    bn_r = addBody_default(skel, bn_r, "leg_r2");
    bn_r = addBody(skel, bn_r, "leg_r3", 
            Vector3d(0, -default_height, 0), Vector3d(0,0,M_PI/2.0), 
            draw, Vector3d(default_width, default_height/4.0, default_depth),
            rest_position, stiffness, damping);
 
    return skel;

}

