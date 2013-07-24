//this one is in basicBayesV3wComm
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BASIC_DEMO_PRIS_H
#define BASIC_DEMO_PRIS_H

#include "LinearMath/btAlignedObjectArray.h"
#include <btBulletDynamicsCommon.h>

#include <stdio.h> //printf debugging
#include <iostream>
#include <fstream>
#include <vector>


///BasicDemo is good starting point for learning the code base and porting.

class BasicDemoPris
{
 public:
  //for printing
  bool debug_print_;
  //keep the collision shapes, for deletion/cleanup
  //btAlignedObjectArray<btCollisionShape*> collisionShapes_;
  
  btBroadphaseInterface* broadphase_;
  
  btDefaultCollisionConfiguration* collisionConfiguration_;

  btCollisionDispatcher* dispatcher_;
  
  btConstraintSolver* solver_;
  
  btDiscreteDynamicsWorld* dynamicsWorld_;
  
  
  //----ADDED
  
  btCollisionShape* groundCollisionShape_;
  btCollisionShape* alphaCollisionShape_;
  btCollisionShape* bravoCollisionShape_;
  btCollisionShape* tangoCollisionShape_;

  btRigidBody* groundRigidBody_;
  btRigidBody* alphaRigidBody_;
  btRigidBody* bravoRigidBody_;
  btRigidBody* tangoRigidBody_;

  btSliderConstraint* sliderConstraint_; //this is the constraint between the fixed point and the sliding beam
  btHingeConstraint* hingeConstraint2_; //this is the constraint between the sliding beam and the robot hand
  
  //to move the object
  btVector3 tangoForce_;
  btVector3 desiredPose_;
  btVector3 startPoseAlpha_;
  btVector3 startPoseBravo_;
  btVector3 startPoseTango_;
  btVector3 pGains_;
  btVector3 dGains_;
  
  //to track the motion
  btTransform tangoBodyTrans_;
  btTransform bravoBodyTrans_;
  btJointFeedback fg_;
  btJointFeedback jfRobot_; //this is the force feedback for the robot
  
  //to do the force following controller
  btScalar currentMoveAngle_;
  btScalar constraintAngle_;
  btVector3 appliedImpulse_;
  btVector3 currentPose_;
  btVector3 currentVel_;
  btScalar count_;
  
  //SAVE IT, BABY!
  std::ofstream myfile;
  
  
  //----ADDED
    
  BasicDemoPris()
    {
      startPoseAlpha_.setValue(0.0f,1.0f,0.0f);
      startPoseBravo_.setValue(0.0f,1.0f,0.0f);
      startPoseTango_.setValue(0.0f,1.0f,0.0f);
    }
  
  virtual ~BasicDemoPris()
    {
      exitPhysics();
    }
  

  void	initPhysics();
  btRigidBody* createRigidBody( btCollisionShape* collisionShape, 
				btScalar mass, 
				const btTransform& transform );
  
  void pController();
  void pdController();
  void stepWorld();
  std::vector<double> runSimulation(std::vector<double> prevState, std::vector<double> action);

  void	exitPhysics();
	
	
};

#endif //BASIC_DEMO_PRIS_H

