#ifndef __BODYNODE_BUILDER_H__
#define __BODYNODE_BUILDER_H__

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
using namespace Eigen;


void drawCylinderJoint(const BodyNodePtr &bn, double width, double depth);

void drawBone(const BodyNodePtr& bn, Vector3d geometry);

BodyNode* makeTranslational2DRootBody(
        const SkeletonPtr& skel, 
        const std::string& name, 
        bool draw, 
        double width, double depth,
        Vector2d rest_position, float stiffness, float damping);

BodyNode* addBody(
        const SkeletonPtr& skel, 
        BodyNode* parent,
        const std::string& name, 
        Vector3d parentBodyToJoint, 
        Vector3d parentBodyToJoint_rot,
        bool draw, 
        Vector3d geometry,
        float rest_position, float stiffness, float damping);

#endif


