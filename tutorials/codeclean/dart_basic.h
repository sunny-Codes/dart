#ifndef __DART_BASIC_H__
#define __DART_BASIC_H__

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
using namespace Eigen;



void setHideOrShow(SkeletonPtr skel, bool hide){
    if(hide){
        for( int i=0; i<skel->getNumShapeNodes(); i++){
            skel->getShapeNode(i)->getVisualAspect()->hide();
        }
    }else{
        for( int i=0; i<skel->getNumShapeNodes(); i++){
            skel->getShapeNode(i)->getVisualAspect()->show();
        }
    }
}
void setColor(BodyNode * bn, Vector3d color){
    for( int i=0; i<bn->getNumShapeNodes(); i++){
        bn->getShapeNode(i)->getVisualAspect()->setColor(color);
    }
}

SkeletonPtr createFloor()
{
    SkeletonPtr floor = Skeleton::create("floor");

    // Give the floor a body
    BodyNodePtr body =
        floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 50.0;
    double floor_height = 1;
    std::shared_ptr<BoxShape> box(
            new BoxShape(Vector3d(floor_width, floor_height, floor_width)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Black());

    // Put the body into position
    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = Vector3d(0.0, -2.6, 0.0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}

void print_bn(BodyNodePtr & bn){
    Isometry3d tf= bn->getWorldTransform();
    Vector4d center= tf.matrix() * Vector4d(0,0,0,1); 
    cout<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
}

void print_Skeleton(SkeletonPtr skel){
    VectorXd pos= skel->getPositions();
    cout<<pos<<endl<<endl;
    int n_bn = skel->getNumBodyNodes();
    for(int i=0; i<n_bn; i++){
        BodyNodePtr bn= skel->getBodyNode(i);
        print_bn(bn);
    }
}


#endif
