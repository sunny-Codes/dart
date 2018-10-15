#include "BodyNodeBuilder.h"

void drawCylinderJoint(const BodyNodePtr &bn, double width, double depth){
    // Make a shape for the Joint
    const double R = width / 2.0;
    const double h = depth;
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    // Line up the cylinder with the Joint axis

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    //tf.linear()= dart::math::eulerXYZToMatrix(Eigen::Vector3d(0,0,30*M_PI/180));
    shapeNode->setRelativeTransform(tf);

}
void drawBone(const BodyNodePtr& bn, Vector3d geometry)
{
      // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(geometry));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  float _height= geometry[1];
  Eigen::Vector3d center = Eigen::Vector3d(0,- _height / 2.0, 0);

  Eigen::Isometry3d tl(Eigen::Isometry3d::Identity());
  tl.translation() = center;
  box_tf= box_tf*tl;

  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNode* makeTranslational2DRootBody(const SkeletonPtr& skel, const std::string& name, bool draw, double width, double depth, Vector2d rest_position, float stiffness, float damping)
{
  // Set up the properties for the Joint
  TranslationalJoint2D::Properties properties;
  properties.mName = name + "_joint";
  properties.setXYPlane();
  //properties.setArbitraryPlane(Vector3d(cos(60*M_PI/180), sin(60*M_PI/180),0),Vector3d(-sin(60*M_PI/180), cos(60*M_PI/180),0) );
  
  properties.mRestPositions= rest_position;

  properties.mSpringStiffnesses[0] = stiffness;
  properties.mDampingCoefficients[0] = damping;

  BodyNodePtr bn = skel->createJointAndBodyNodePair<TranslationalJoint2D>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  if(draw) drawCylinderJoint(bn,width, depth);

  return bn;
}


BodyNode* addBody(
        const SkeletonPtr& skel, 
        BodyNode* parent,
        const string& name, 
        Vector3d parentBodyToJoint, 
        Vector3d parentBodyToJoint_rot,
        bool draw, 
        Vector3d geometry,
        float rest_position, float stiffness, float damping)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitZ();
  Eigen::Isometry3d tf_1(Eigen::Isometry3d::Identity());
  tf_1.linear()= dart::math::eulerXYZToMatrix(parentBodyToJoint_rot);

  properties.mT_ParentBodyToJoint= tf_1;
  
  properties.mT_ParentBodyToJoint.translation() = parentBodyToJoint;
 
  properties.mRestPositions[0] = rest_position;
  properties.mSpringStiffnesses[0] = stiffness;
  properties.mDampingCoefficients[0] = damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = skel->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  if(draw){
      drawCylinderJoint(bn, geometry[0], geometry[2]);
      drawBone(bn, geometry);
  }
  return bn;
}


