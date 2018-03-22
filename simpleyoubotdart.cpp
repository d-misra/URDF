/*
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
#include <dart/utils/urdf/urdf.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

//const double default_height = 1.0; // m
//const double default_width = 0.2;  // m
//const double default_depth = 0.2;  // m

//const double default_torque = 15.0; // N-m
//const double default_force =  15.0; // N
//const int default_countdown = 200;  // Number of timesteps for applying force

//const double default_rest_position = 0.0;
//const double delta_rest_position = 10.0 * M_PI / 180.0;

//const double default_stiffness = 0.0;
//const double delta_stiffness = 10;

//const double default_damping = 5.0;
//const double delta_damping = 1.0;

//using namespace dart::dynamics;
//using namespace dart::simulation;

//class MyWindow : public dart::gui::SimWindow
//{
//public:

//  /// Constructor
//  MyWindow(WorldPtr world)
//    : mBallConstraint(nullptr),
//      mPositiveSign(true),
//      mBodyForce(false)
//  {
//    setWorld(world);

//    // Find the Skeleton named "youbot" within the World
//    mYoubot = world->getSkeleton("youbot");

//    // Make sure that the youbot was found in the World
//    assert(mYoubot != nullptr);

//    mForceCountDown.resize(mYoubot->getNumDofs(), 0);

//    ArrowShape::Properties arrow_properties;
//    arrow_properties.mRadius = 0.05;
//    mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
//             Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
//             Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
//             arrow_properties, dart::Color::Orange(1.0)));
//  }


//int main(int argc, char* argv[])
//{
//  SkeletonPtr youbot = Skeleton::create("youbot");
//  
//}

//SkeletonPtr createManipulator()
//{
//  // Load the Skeleton from a file
//  dart::utils::DartLoader loader;
//  SkeletonPtr manipulator = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
//  manipulator->setName("manipulator");

//  // Position its base in a reasonable way
//  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
//  tf.translation() = Eigen::Vector3d(-0.65, 0.0, 0.0);
//  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

//  // Get it into a useful configuration
//  manipulator->getDof(1)->setPosition(140.0 * M_PI / 180.0);
//  manipulator->getDof(2)->setPosition(-140.0 * M_PI / 180.0);

//  return manipulator;
//}

//SkeletonPtr createYoubot()
//{
// 
// // Load the Skeleton from a file
// dart::utils::DartLoader loader;
// loader.dart::utils::DartLoader::addPackageDirectory("youbot-ros-pkg-kdl","/home/deba/Documents/Practice/dart/data/urdfyoubot/youbot-ros-pkg-kdl");
// SkeletonPtr youbot =  loader.parseSkeleton("/home/deba/Documents/Practice/dart/data/urdfyoubot/youbot-ros-pkg-kdl/robots/youbot.urdf");
// youbot->setName("youbot"); 
// return youbot;
//}

//SkeletonPtr createArm()
//{
// 
// // Load the Skeleton from a file
// dart::utils::DartLoader loader;
// SkeletonPtr arm =  loader.parseSkeleton("/home/deba/Code/source/robot_dart/res/models/arm.urdf");
// arm->setName("arm"); 
// return arm;
//}

class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(const WorldPtr& world)
  {
    setWorld(world);
  }

};

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
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  return floor;
}

SkeletonPtr createYoubot()
{
 
 // Load the Skeleton from a file
 dart::utils::DartLoader loader;
 SkeletonPtr youbot =  loader.parseSkeleton("/home/deba/Code/limbo/exp/blackdrops/res/Simpleyoubot/youbot.urdf");
 youbot->setName("youbot"); 
 return youbot;
}

SkeletonPtr createArm()
{
 
 // Load the Skeleton from a file
 dart::utils::DartLoader loader;
 //SkeletonPtr arm =  loader.parseSkeleton("/home/deba/Code/limbo/exp/blackdrops/res/Simpleyoubot/arm.urdf");
 SkeletonPtr arm =  loader.parseSkeleton("/home/deba/Code/limbo/exp/blackdrops/arm_models/URDF/omnigrasper_hook_real.urdf");
 arm->setName("arm"); 
 return arm;
}

SkeletonPtr createArm2()
{
 
 // Load the Skeleton from a file
 dart::utils::DartLoader loader;
 SkeletonPtr arm2 =  loader.parseSkeleton("/home/deba/Code/limbo/exp/blackdrops/res/Simpleyoubot/armtest.urdf");
 arm2->setName("arm2"); 
 return arm2;
}


SkeletonPtr createYoubotbase()
{
 
 // Load the Skeleton from a file
 dart::utils::DartLoader loader;
 SkeletonPtr youbot_base =  loader.parseSkeleton("/home/deba/Code/limbo/exp/blackdrops/res/Simpleyoubot/youbot2.urdf");
 youbot_base->setName("youbot_base"); 
 return youbot_base;
}

SkeletonPtr createYoubotall()
{
 
 // Load the Skeleton from a file
 dart::utils::DartLoader loader;
 SkeletonPtr youbot_all =  loader.parseSkeleton("/home/deba/Code/limbo/exp/blackdrops/res/Simpleyoubot/arm_ybt.urdf"); //youbot4.urdf works fine - merger of the existing three link and base
 youbot_all->setName("youbot_all"); 
 return youbot_all;
}

int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();
  SkeletonPtr youbot = createYoubot();
  SkeletonPtr arm = createArm();
  SkeletonPtr arm2 = createArm2();
  SkeletonPtr youbot_base = createYoubotbase();
  SkeletonPtr youbot_all = createYoubotall();

//  SkeletonPtr arm = createArm();
//  SkeletonPtr manipulator = createManipulator();
    
  WorldPtr world = std::make_shared<World>();
  //world->addSkeleton(youbot);
 world->addSkeleton(arm);
//world->addSkeleton(arm2);
//  world->addSkeleton(youbot_base);
 //world->addSkeleton(youbot_all);
  world->addSkeleton(floor);

//  world->addSkeleton(manipulator);
//  world->addSkeleton(arm);

  MyWindow window(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Youbot");
  glutMainLoop();
}
