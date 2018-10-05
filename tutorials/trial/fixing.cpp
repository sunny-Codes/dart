/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#define EPSILON 0.0001
const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = EPSILON;  // m

const double default_torque = 15.0; // N-m
const double default_force =  15.0; // N
const int default_countdown = 200;  // Number of timesteps for applying force

const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

// desired 
const double desired_height = 1.0; // m
const double desired_width = 0.2;  // m
const double desired_depth = 0.2;  // m
const double desired_rest_position = 0.0;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9*default_ground_width/2;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
using namespace Eigen;
// goal position
int tot_dof=10; // root(translational) 2 + revolute joint angle 4 * 2(left,right)
int P_NUM=3; //number of goal poses
int cur_p_idx;
VectorXd goalPos(tot_dof);
MatrixXd goalPoses(tot_dof,P_NUM);
VectorXd goalPos_0(tot_dof);
VectorXd goalPos_1(tot_dof);
VectorXd goalPos_2(tot_dof);


class MyWindow : public dart::gui::SimWindow
{
public:

  /// Constructor
  MyWindow(WorldPtr world)
    : mBallConstraint(nullptr),
      mPositiveSign(true),
      mBodyForce(false),
      mPD(false)
  {
    setWorld(world);

    // Find the Skeleton named "pendulum" within the World
    mPendulum = world->getSkeleton("pendulum");

    // Make sure that the pendulum was found in the World
    assert(mPendulum != nullptr);

    mForceCountDown.resize(mPendulum->getNumDofs(), 0);

    ArrowShape::Properties arrow_properties;
    arrow_properties.mRadius = 0.05;
    mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
             Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
             Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
             arrow_properties, dart::Color::Orange(1.0)));

    // Set PD control gains
    mKpPD = 200.0;
    mKdPD = 20.0;
  }

  void changeDirection()
  {
    mPositiveSign = !mPositiveSign;
    if(mPositiveSign)
    {
      mArrow->setPositions(
            Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
            Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0));
    }
    else
    {
      mArrow->setPositions(
            Eigen::Vector3d(default_height, 0.0, default_height / 2.0),
            Eigen::Vector3d(default_width / 2.0, 0.0, default_height / 2.0));
    }
  }

  void applyForce(std::size_t index)
  {
    if(index < mForceCountDown.size())
      mForceCountDown[index] = default_countdown;
  }

  void changeRestPosition(double delta)
  {
    for(std::size_t i = 0; i < mPendulum->getNumDofs(); ++i)
    {
      DegreeOfFreedom* dof = mPendulum->getDof(i);
      double q0 = dof->getRestPosition() + delta;
      cout<<i<<"\t"<<dof->getRestPosition()<<"\t"<<q0<<endl;
      // The system becomes numerically unstable when the rest position exceeds
      // 90 degrees
      if(std::abs(q0) > 90.0 * M_PI / 180.0)
        q0 = (q0 > 0)? (90.0 * M_PI / 180.0) : -(90.0 * M_PI / 180.0);

      dof->setRestPosition(q0);
    }

    // Only curl up along one axis in the BallJoint
    // mPendulum->getDof(0)->setRestPosition(0.0);
    // mPendulum->getDof(2)->setRestPosition(0.0);
  }

  void changeStiffness(double delta)
  {
    for(std::size_t i = 0; i < mPendulum->getNumDofs(); ++i)
    {
      DegreeOfFreedom* dof = mPendulum->getDof(i);
      double stiffness = dof->getSpringStiffness() + delta;
      if(stiffness < 0.0)
        stiffness = 0.0;
      dof->setSpringStiffness(stiffness);
    }
  }

  void changeDamping(double delta)
  {
    for(std::size_t i = 0; i < mPendulum->getNumDofs(); ++i)
    {
      DegreeOfFreedom* dof = mPendulum->getDof(i);
      double damping = dof->getDampingCoefficient() + delta;
      if(damping < 0.0)
        damping = 0.0;
      dof->setDampingCoefficient(damping);
    }
  }

  /// Add a constraint to attach the final link to the world
  void addConstraint()
  {
    cout<<"addConstraint entered"<<endl;
    // Get the last body in the pendulum
    BodyNode* tip  = mPendulum->getBodyNode(mPendulum->getNumBodyNodes() - 1);

    // Attach the last link to the world
    Eigen::Vector3d location =
        tip->getTransform() * Eigen::Vector3d(0.0, 0.0, default_height);
    mBallConstraint =
        std::make_shared<dart::constraint::BallJointConstraint>(tip, location);
    mWorld->getConstraintSolver()->addConstraint(mBallConstraint);
  }

  /// Remove any existing constraint, allowing the pendulum to flail freely
  void removeConstraint()
  {
    cout<<"removeConstraint entered"<<endl;
    mWorld->getConstraintSolver()->removeConstraint(mBallConstraint);
    mBallConstraint = nullptr;
  }

  void setPDForces()
  {
    if(nullptr == mPendulum)
      return;

    // Compute the joint position error
    Eigen::VectorXd q = mPendulum->getPositions();
    Eigen::VectorXd dq = mPendulum->getVelocities();
    q += dq * mPendulum->getTimeStep();

    Eigen::VectorXd q_err = goalPos - q;

    // Compute the joint velocity error
    Eigen::VectorXd dq_err = -dq;

    // Compute the joint forces needed to compensate for Coriolis forces and
    // gravity
    
    //const Eigen::VectorXd& Cg = mPendulum->getCoriolisAndGravityForces();

    // Compute the desired joint forces
    const Eigen::MatrixXd& M = mPendulum->getMassMatrix();
    mForces = M * (mKpPD * q_err + mKdPD * dq_err) ; //+ Cg;

    mPendulum->setForces(mForces);
  }


  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case '-':
        changeDirection();
        break;

      case '1':
        applyForce(0);
        break;
      case '2':
        applyForce(1);
        break;
      case '3':
        applyForce(2);
        break;
      case '4':
        applyForce(3);
        break;
      case '5':
        applyForce(4);
        break;
      case '6':
        applyForce(5);
        break;
      case '7':
        applyForce(6);
        break;
      case '8':
        applyForce(7);
        break;
      case '9':
        applyForce(8);
        break;
      case '0':
        mPD= !mPD;
        //setPDForces();
        //applyForce(9);
        break;
      case 'c':
        cur_p_idx= (cur_p_idx+1)% P_NUM;
        cout<<"cur_p_idx"<<endl;
        goalPos= goalPoses.col(cur_p_idx);
        cout<<goalPos.transpose()<<endl;
        break;

      case 'q':
        changeRestPosition(delta_rest_position);
        break;
      case 'a':
        changeRestPosition(-delta_rest_position);
        break;

      case 'w':
        changeStiffness(delta_stiffness);
        break;
      case 's':
        changeStiffness(-delta_stiffness);
        break;

      case 'e':
        changeDamping(delta_damping);
        break;
      case 'd':
        changeDamping(-delta_damping);
        break;

      case 'r':
      {
        if(mBallConstraint)
          removeConstraint();
        else
          addConstraint();
        break;
      }

      case 'f':
        mBodyForce = !mBodyForce;
        break;

      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    // Reset all the shapes to be Blue
    // lesson 1-a
    for(std::size_t i = 0; i < mPendulum->getNumBodyNodes(); ++i)
    {
      BodyNode* bn = mPendulum->getBodyNode(i);
      auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
      //for(std::size_t j = 0; j < 1; ++j)

      visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Green());
      if(visualShapeNodes.size()>1){
          visualShapeNodes[1]->getVisualAspect()->setColor(dart::Color::Orange());
      }       

      // If we have three visualization shapes, that means the arrow is
      // attached. We should remove it in case this body is no longer
      // experiencing a force
      if(visualShapeNodes.size() == 3u)
      {
        assert(visualShapeNodes[2]->getShape() == mArrow);
        visualShapeNodes[2]->remove();
      }
    }
    
    
    if(mPD){
        setPDForces();
    }


    if(!mBodyForce)
    {
      // Apply joint torques based on user input, and color the Joint shape red
      for(std::size_t i = 0; i < mPendulum->getNumDofs(); ++i)
      {
        if(mForceCountDown[i] > 0)
        {
          DegreeOfFreedom* dof = mPendulum->getDof(i);
          dof->setForce( mPositiveSign? default_torque : -default_torque );

          BodyNode* bn = dof->getChildBodyNode();
          auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
          visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

          --mForceCountDown[i];
        }
      }
    }
    else
    {
      // Apply body forces based on user input, and color the body shape red
      for(std::size_t i = 0; i < mPendulum->getNumBodyNodes(); ++i)
      {
        if(mForceCountDown[i] > 0)
        {
          BodyNode* bn = mPendulum->getBodyNode(i);

          Eigen::Vector3d force = default_force * Eigen::Vector3d::UnitX();
          Eigen::Vector3d location(-default_width / 2.0, 0.0, default_height / 2.0);
          if(!mPositiveSign)
          {
            force = -force;
            location[0] = -location[0];
          }
          bn->addExtForce(force, location, true, true);

          auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
          shapeNodes[1]->getVisualAspect()->setColor(dart::Color::Red());
          bn->createShapeNodeWith<VisualAspect>(mArrow);

          --mForceCountDown[i];
        }
      }
    }
    
    
    // Step the simulation forward
    SimWindow::timeStepping();
    for (int i=0;i<tot_dof;i++){
        //if(i==2) mWorld->getSkeleton("goal_pendulum")->getRootBodyNode()->setProperties
        mWorld->getSkeleton("goal_pendulum")->setPosition(i, goalPos[i]);
    }
  }

protected:

  /// An arrow shape that we will use to visualize applied forces
  std::shared_ptr<ArrowShape> mArrow;

  /// The pendulum that we will be perturbing
  SkeletonPtr mPendulum;

  /// Pointer to the ball constraint that we will be turning on and off
  dart::constraint::BallJointConstraintPtr mBallConstraint;

  /// Number of iterations before clearing a force entry
  std::vector<int> mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;

  /// True if 1-9 should be used to apply a body force. Otherwise, 1-9 will be
  /// used to apply a joint torque.
  bool mBodyForce;

  /// Control gains for the proportional error terms in the PD controller
  double mKpPD;

  /// Control gains for the derivative error terms in the PD controller
  double mKdPD;
  
  /// Joint forces for the manipulator (output of the Controller)
  Eigen::VectorXd mForces;

  // True if using PD control (change by '0' keyboard)
  bool mPD; 

};

void setGeometry(const BodyNodePtr& bn, float _width=default_width, float _depth=default_depth, float _height=default_height)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(_width, _height, _depth)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0,- _height / 2.0, 0);
  //box_tf.linear()= dart::math::eulerXYZToMatrix(
  //        Eigen::Vector3d(0,0,30*M_PI/180));

  Eigen::Isometry3d tl(Eigen::Isometry3d::Identity());
  tl.translation() = center;
  box_tf= box_tf*tl;

  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNode* makeRevoluteRootBody(const SkeletonPtr& pendulum, const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitZ();

  properties.mRestPositions[0] = default_rest_position;
  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis

Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  //tf.linear()= dart::math::eulerXYZToMatrix(Eigen::Vector3d(0,0,30*M_PI/180));
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

BodyNode* makeTranslational2DRootBody(const SkeletonPtr& pendulum, const std::string& name)
{
  // Set up the properties for the Joint
  TranslationalJoint2D::Properties properties;
  properties.mName = name + "_joint";
  properties.setXYPlane();
  //properties.setArbitraryPlane(Vector3d(cos(60*M_PI/180), sin(60*M_PI/180),0),Vector3d(-sin(60*M_PI/180), cos(60*M_PI/180),0) );
  
  properties.mRestPositions[0] = default_rest_position;
  properties.mRestPositions[1] = default_rest_position;

  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<TranslationalJoint2D>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  //tf.linear()= dart::math::eulerXYZToMatrix(Eigen::Vector3d(0,0,30*M_PI/180));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  //setGeometry(bn);

  return bn;
}


BodyNode* addBody(const SkeletonPtr& pendulum, BodyNode* parent,
                  const std::string& name, Eigen::Vector3d parentBodyToJoint= Vector3d(0,- default_height, 0))
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitZ();
  Eigen::Isometry3d tf_1(Eigen::Isometry3d::Identity());
  properties.mT_ParentBodyToJoint= tf_1;
  
  //ADDED
  /*tf_1.linear()= dart::math::eulerXYZToMatrix(
          Eigen::Vector3d(0,0,30*M_PI/180));
  Eigen::Isometry3d tf_2(Eigen::Isometry3d::Identity());
  tf_2.translation()= Eigen::Vector3d(0,-default_height,0);
  properties.mT_ParentBodyToJoint= tf_1*tf_2;
  */
  properties.mT_ParentBodyToJoint.translation() = parentBodyToJoint;
      //Eigen::Vector3d(0,-default_height,0);
  properties.mRestPositions[0] = default_rest_position;
  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

/*
SkeletonPtr createBall()
{
  SkeletonPtr ball = Skeleton::create("rigid_ball");

  // Give the ball a body
  addRigidBody<FreeJoint>(ball, "rigid ball", Shape::ELLIPSOID);

  setAllColors(ball, dart::Color::Red());
  
  // Set the starting position for the object
  Eigen::Vector6d positions(Eigen::Vector6d::Zero());
  positions[4]=10;
  positions[5]=20;
  ball->getJoint(0)->setPositions(positions);

  return ball;
}
*/

void print_bn(BodyNodePtr & bn){
    Eigen::Isometry3d tf= bn->getWorldTransform();
    Eigen::Vector4d center= tf.matrix() * Eigen::Vector4d(0,0,0,1); 
    cout<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
}

void print_Skeleton(SkeletonPtr skel){
    Eigen::VectorXd pos= skel->getPositions();
    cout<<pos<<endl<<endl;
    int n_bn = skel->getNumBodyNodes();
    for(int i=0; i<n_bn; i++){
        BodyNodePtr bn= skel->getBodyNode(i);
        print_bn(bn);
    }
}


SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_ground_width, default_ground_width,
                        default_wall_thickness));
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 1.0, 1.0));

  return ground;
}


SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");
  
  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  
  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());
  
  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -5.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  
  return floor;
}



int main(int argc, char* argv[])
{

    goalPos_0 << 0, 0, 30*M_PI/180, 0, 0, 0;
    goalPos_1 << 0, 0, 0, 30*M_PI/180.0,  0, 0;
    goalPos_2 << 0, 0, 0, 0, 30*M_PI/180.0, 0;
    
    goalPoses << goalPos_0, goalPos_1, goalPos_2;
    cur_p_idx=0;
    goalPos= goalPoses.col(cur_p_idx); //goalPos_1;


    // Create an empty Skeleton with the name "pendulum"
  SkeletonPtr pendulum = Skeleton::create("pendulum");

  // Add each body to the last BodyNode in the pendulum
  BodyNode* bn = makeTranslational2DRootBody(pendulum, "root");
  BodyNode * bn_l = addBody(pendulum, bn, "body_l1", Eigen::Vector3d(0,0,0));
  bn_l = addBody(pendulum, bn_l, "body_l2");
  bn_l = addBody(pendulum, bn_l, "body_l3");
  bn_l = addBody(pendulum, bn_l, "body_l4");
  
  BodyNode* bn_r = addBody(pendulum, bn, "body_r1", Eigen::Vector3d(0,0,0));
  bn_r = addBody(pendulum, bn_r, "body_r2");
  bn_r = addBody(pendulum, bn_r, "body_r3");
  bn_r = addBody(pendulum, bn_r, "body_r4");

  // Set the initial position of the first DegreeOfFreedom so that the pendulum
  // starts to swing right away
  pendulum->getDof(0)->setPosition(100 * M_PI / 180.0);

  // Create a goal pendulum
  SkeletonPtr g_pendulum = Skeleton::create("goal_pendulum");
  BodyNode* g_bn = makeTranslational2DRootBody(g_pendulum, "goal_root");
  
  BodyNode* g_bn_l = addBody(g_pendulum, g_bn, "goalbody_l1", Eigen::Vector3d(0,0,0));
  g_bn_l = addBody(g_pendulum, g_bn_l, "goalbody_l2");
  g_bn_l = addBody(g_pendulum, g_bn_l, "goalbody_l3");
  g_bn_l = addBody(g_pendulum, g_bn_l, "goalbody_l4");
 
  BodyNode* g_bn_r = addBody(g_pendulum, g_bn, "goalbody_r1", Eigen::Vector3d(0,0,0));
  g_bn_r = addBody(g_pendulum, g_bn_r, "goalbody_r2");
  g_bn_r = addBody(g_pendulum, g_bn_r, "goalbody_r3");
  g_bn_r = addBody(g_pendulum, g_bn_r, "goalbody_r4");
  
  //totdof=10 //cout<<g_pendulum->getNumDofs()<<endl;


  for (int i=0;i<g_pendulum->getNumBodyNodes();i++){
      g_pendulum->getBodyNode(i)->setCollidable(false);
  }

  // Create a world and add the pendulum to the world
  WorldPtr world= std::make_shared<World>();

  if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet"))
  {
      world->getConstraintSolver()->setCollisionDetector(
              dart::collision::CollisionDetector::getFactory()->create("bullet"));
  }


  world->addSkeleton(pendulum);
  world->addSkeleton(g_pendulum);
  world->addSkeleton(createFloor());

  world->setGravity(Vector3d(0,-9.8,0));
  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  print_Skeleton(pendulum);
  // Print instructions
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'1' -> '9': apply torque to a pendulum body" << std::endl;
  std::cout << "'-': Change sign of applied joint torques" << std::endl;
  std::cout << "'q': Increase joint rest positions" << std::endl;
  std::cout << "'a': Decrease joint rest positions" << std::endl;
  std::cout << "'w': Increase joint spring stiffness" << std::endl;
  std::cout << "'s': Decrease joint spring stiffness" << std::endl;
  std::cout << "'e': Increase joint damping" << std::endl;
  std::cout << "'d': Decrease joint damping" << std::endl;
  std::cout << "'r': add/remove constraint on the end of the chain" << std::endl;
  std::cout << "'f': switch between applying joint torques and body forces" << std::endl;

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(1080, 810, "2D walking body");
  glutMainLoop();
}
