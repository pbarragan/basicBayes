//this one is in newTestSlidingBox

#include <iostream>
#include "BasicDemo.h"
#include <btBulletDynamicsCommon.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>

//Creates a rigid body I think
btRigidBody* BasicDemo::createRigidBody( btCollisionShape* collisionShape, 
			      btScalar mass, 
			      const btTransform& transform ){
  // calculate inertia
  btVector3 localInertia( 0.0f, 0.0f, 0.0f );
  collisionShape->calculateLocalInertia( mass, localInertia );    
  
  // create motion state
  btDefaultMotionState* defaultMotionState 
    = new btDefaultMotionState( transform );
  
  // create rigid body
  btRigidBody::btRigidBodyConstructionInfo rigidBodyConstructionInfo( 
								     mass, defaultMotionState, collisionShape, localInertia );
  btRigidBody* rigidBody = new btRigidBody( rigidBodyConstructionInfo );      
  
  return rigidBody;
}

void BasicDemo::pdController(){
  
  //appliedImpulse = hingeConstraint2->getJointFeedback()->m_appliedForceBodyB;
  //std::cout << "x force " << appliedImpulse.x() << std::endl;
  //std::cout << "y force " << appliedImpulse.y() << std::endl;
  //std::cout << "z force " << appliedImpulse.z() << std::endl;

  //get position
  tangoRigidBody_->getMotionState()->getWorldTransform(tangoBodyTrans_);
  currentPose_ = tangoBodyTrans_.getOrigin();
  //get velocity
  currentVel_ = tangoRigidBody_->getLinearVelocity();

  tangoForce_ = pGains_*(desiredPose_ - tangoBodyTrans_.getOrigin()) - dGains_*currentVel_;
  tangoRigidBody_->applyCentralForce( tangoForce_ );
  
}

void BasicDemo::pController(){
  
  //appliedImpulse = hingeConstraint2->getJointFeedback()->m_appliedForceBodyB;
  //std::cout << "x force " << appliedImpulse.x() << std::endl;
  //std::cout << "y force " << appliedImpulse.y() << std::endl;
  //std::cout << "z force " << appliedImpulse.z() << std::endl;

  //get position
  tangoRigidBody_->getMotionState()->getWorldTransform(tangoBodyTrans_);
  currentPose_ = tangoBodyTrans_.getOrigin();

  tangoForce_ = pGains_*(desiredPose_ - tangoBodyTrans_.getOrigin());
  tangoRigidBody_->applyCentralForce( tangoForce_ );

  
}

void BasicDemo::stepWorld(){
    
  pdController();

  dynamicsWorld_->stepSimulation(1/20.f,10);
  
  std::cout << "hand X position: " << currentPose_.getX() << std::endl;
  std::cout << "hand Y position: " << currentPose_.getY() << std::endl;
  std::cout << "hand Z position: " << currentPose_.getZ() << std::endl;
  
  bravoRigidBody_->getMotionState()->getWorldTransform(bravoBodyTrans_);
  std::cout << "box X position: " << bravoBodyTrans_.getOrigin().getX() << std::endl;
  std::cout << "box Y position: " << bravoBodyTrans_.getOrigin().getY() << std::endl;
  std::cout << "box Z position: " << bravoBodyTrans_.getOrigin().getZ() << std::endl;
}

void BasicDemo::initPhysics(){

  //set up world
  broadphase_ = new btDbvtBroadphase();
  
  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
  
  solver_ = new btSequentialImpulseConstraintSolver;
  
  dynamicsWorld_ = new btDiscreteDynamicsWorld(dispatcher_,broadphase_,solver_,collisionConfiguration_);

  //set gravity. y is apparently up.
  dynamicsWorld_->setGravity(btVector3(0,-10,0));

  //--------------OBJECT CREATION SECTION-----------------//

  //make the ground plane
  groundCollisionShape_ = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
  //collisionShapes_.push_back(groundCollisionShape_);
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0.0f,-50.0f,0.0f));
  btScalar groundMass(0.0);

  //create the ground plane rigid body
  groundRigidBody_ = createRigidBody(groundCollisionShape_,groundMass,groundTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(groundRigidBody_);

  //Calculate the required origins based on startPose_ given by the runSimulation function
  btVector3 bravoOrigin = startPose_;
  btVector3 tangoOrigin = startPose_;
  tangoOrigin.setX(tangoOrigin.getX()+1.5);
  
  //make bravo rigid body (the sliding box)
  const btVector3 bravoBoxHalfExtents( 1.0f, 1.0f, 1.0f );
  bravoCollisionShape_ = new btBoxShape(bravoBoxHalfExtents);
  //collisionShapes_.push_back(bravoCollisionShape_);
  btTransform bravoTransform;
  bravoTransform.setIdentity();
  //bravoTransform.setOrigin(btVector3(0.0f,1.0f,0.0f));
  bravoTransform.setOrigin(bravoOrigin);
  btScalar bravoMass = 1.0f;

  //create the bravo rigid body
  bravoRigidBody_ = createRigidBody(bravoCollisionShape_,bravoMass,bravoTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(bravoRigidBody_);

  //make tango rigid body (the robots manipulator)
  const btVector3 tangoBoxHalfExtents( 0.1f, 0.5f, 0.1f );
  tangoCollisionShape_ = new btBoxShape(tangoBoxHalfExtents);
  //collisionShapes_.push_back(tangoCollisionShape_);
  btTransform tangoTransform;
  tangoTransform.setIdentity();
  //tangoTransform.setOrigin(btVector3(1.5f,1.0f,0.0f));
  tangoTransform.setOrigin(tangoOrigin);
  btScalar tangoMass = 0.1f;

  //create the tango rigid body
  tangoRigidBody_ = createRigidBody(tangoCollisionShape_,tangoMass,tangoTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(tangoRigidBody_);

  //--------------END OBJECT CREATION SECTION-----------------//

  //--------------CONSTRAINT CREATION SECTION-----------------//
  
  // create a constraint
  const btVector3 pivotInA( 1.5f, 0.0f, 0.0f );   
  const btVector3 pivotInB( 0.0f, 0.0f, 0.0f );
  btVector3 axisInA( 0.0f, 1.0f, 0.0f );
  btVector3 axisInB( 0.0f, 1.0f, 0.0f );
  bool useReferenceFrameA = false;
  hingeConstraint_ = new btHingeConstraint(*bravoRigidBody_,*tangoRigidBody_,pivotInA,pivotInB,axisInA,axisInB,useReferenceFrameA);
  
  // set joint feedback
  hingeConstraint_->setJointFeedback(&jfRobot_);
  // add constraint to the world
  const bool isDisableCollisionsBetweenLinkedBodies = false;
  dynamicsWorld_->addConstraint(hingeConstraint_, 
				  isDisableCollisionsBetweenLinkedBodies);
  
  //--------------END CONSTRAINT CREATION SECTION-----------------//

  //--------------INITIALIZE REMAINING PARAMETERS SECTION-----------------//
  
  //set controller values
  pGains_.setValue(5.0f,5.0f,5.0f);
  dGains_.setValue(2.0f,2.0f,2.0f);
  
  //initialize
  tangoRigidBody_->getMotionState()->getWorldTransform(tangoBodyTrans_);
  currentPose_ = tangoBodyTrans_.getOrigin();
  desiredPose_ = currentPose_;
  bravoRigidBody_->forceActivationState(4);
  count_ = 0;
  
  //--------------END INITIALIZE REMAINING PARAMETERS SECTION-----------------//
  
}

void BasicDemo::exitPhysics(){

  dynamicsWorld_->removeRigidBody(groundRigidBody_);
  delete groundRigidBody_->getMotionState();
  delete groundRigidBody_;
  
  dynamicsWorld_->removeRigidBody(bravoRigidBody_);
  delete bravoRigidBody_->getMotionState();
  delete bravoRigidBody_;

  dynamicsWorld_->removeRigidBody(tangoRigidBody_);
  delete tangoRigidBody_->getMotionState();
  delete tangoRigidBody_;
  
  
  delete groundCollisionShape_;
  delete bravoCollisionShape_;
  delete tangoCollisionShape_;

  delete dynamicsWorld_;

  delete solver_;
  delete collisionConfiguration_;
  delete dispatcher_;
  delete broadphase_;
}

std::vector<double> BasicDemo::runSimulation(std::vector<double> prevState, std::vector<double> action){
  startPose_.setValue(prevState[0],1.0,prevState[1]);
  initPhysics();
  desiredPose_.setValue(action[0],1.0,action[1]);

  for(size_t i = 0; i<3000;i++){
    stepWorld();
  }

  std::vector<double> nextState;
  nextState.push_back(bravoBodyTrans_.getOrigin().getX());
  nextState.push_back(bravoBodyTrans_.getOrigin().getZ());

  return nextState;
}

int main (void)
{
 
  BasicDemo world;
  
  std::vector<double> prevState;
  prevState.push_back(0.0);
  prevState.push_back(0.0);

  std::vector<double> action;
  action.push_back(1.0);
  action.push_back(1.0);

  std::vector<double> nextState = world.runSimulation(prevState,action);

  std::cout << "ans:" << std::endl;
  std::cout << nextState[0] << "," << nextState[1] << std::endl;
  
  /*
  world.initPhysics();
  world.desiredPose_.setValue(10.0f,1.0f,10.0f);
  for(size_t i = 0; i<3000;i++){
    world.stepWorld();
  }
  //world.exitPhysics();
  //delete &world;
  */
  std::cout << "I made it TO THE END" << std::endl;

  return 0;
}

