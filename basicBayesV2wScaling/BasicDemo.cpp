//this one is in basicBayesV2wScaling

#include <iostream>
#include "BasicDemo.h"
//#include <btBulletDynamicsCommon.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>

//scaling factor defined in BasicDemo.h
//#define SCALING 1.

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
  std::cout << "I created a rigid body" << std::endl;
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
  if(debug_print_){
    std::cout << "force applied:" << std::endl;
    std::cout << tangoForce_[0] << std::endl;
    std::cout << tangoForce_[1] << std::endl;
    std::cout << tangoForce_[2] << std::endl;
  }
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
  //std::cout << "Activation state: " << std::endl;
  //std::cout << tangoRigidBody_->getActivationState() << std::endl;
    
  pdController();

  //std::cout << "Activation state: " << std::endl;
  //std::cout << bravoRigidBody_->getActivationState() << std::endl;


  //dynamicsWorld_->stepSimulation(1./240.f,1,1./240.f);
  dynamicsWorld_->stepSimulation(1./60.f);

  //std::cout << "Activation state: " << std::endl;
  //std::cout << tangoRigidBody_->getActivationState() << std::endl;


  bravoRigidBody_->getMotionState()->getWorldTransform(bravoBodyTrans_);

  //std::cout << "Activation state: " << std::endl;
  //std::cout << bravoRigidBody_->getActivationState() << std::endl;

  
  if(debug_print_){
    std::cout << "hand X position: " << currentPose_.getX() << std::endl;
    std::cout << "hand Y position: " << currentPose_.getY() << std::endl;
    std::cout << "hand Z position: " << currentPose_.getZ() << std::endl;
    
    
    std::cout << "box X position: " << bravoBodyTrans_.getOrigin().getX() << std::endl;
    std::cout << "box Y position: " << bravoBodyTrans_.getOrigin().getY() << std::endl;
    std::cout << "box Z position: " << bravoBodyTrans_.getOrigin().getZ() << std::endl;
  }
  

}

void BasicDemo::initPhysics(){

  //printing
  debug_print_ = false;

  //set up world
  broadphase_ = new btDbvtBroadphase();
  
  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
  
  solver_ = new btSequentialImpulseConstraintSolver;
  
  dynamicsWorld_ = new btDiscreteDynamicsWorld(dispatcher_,broadphase_,solver_,collisionConfiguration_);

  //set gravity. y is apparently up.
  dynamicsWorld_->setGravity(btVector3(0,-10*SCALING,0));

  //--------------OBJECT CREATION SECTION-----------------//

  //make the ground plane
  groundCollisionShape_ = new btBoxShape(btVector3(btScalar(50.0),btScalar(50.0),btScalar(50.0)));
  //collisionShapes_.push_back(groundCollisionShape_);
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0.0f,-50.0f,0.0f));
  btScalar groundMass(0.0);

  //create the ground plane rigid body
  //groundCollisionShape_->setMargin(0.004);
  groundRigidBody_ = createRigidBody(groundCollisionShape_,groundMass,groundTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(groundRigidBody_);

  //Calculate the required origins based on startPose_ given by the runSimulation function
  btVector3 bravoOrigin = startPose_;
  btVector3 tangoOrigin = startPose_;
  //tangoOrigin.setX(tangoOrigin.getX()+1.5);
  
  //make bravo rigid body (the sliding box)
  const btVector3 bravoBoxHalfExtents( 1.0f*SCALING, 1.0f*SCALING, 1.0f*SCALING );
  bravoCollisionShape_ = new btBoxShape(bravoBoxHalfExtents);
  //collisionShapes_.push_back(bravoCollisionShape_);
  btTransform bravoTransform;
  bravoTransform.setIdentity();
  //bravoTransform.setOrigin(btVector3(0.0f,1.0f,0.0f));
  bravoTransform.setOrigin(bravoOrigin);
  btScalar bravoMass = 1.0f;

  //create the bravo rigid body
  //bravoCollisionShape_->setMargin(0.004);
  bravoRigidBody_ = createRigidBody(bravoCollisionShape_,bravoMass,bravoTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(bravoRigidBody_);

  //make tango rigid body (the robots manipulator)
  const btVector3 tangoBoxHalfExtents( 0.1f*SCALING, 0.5f*SCALING, 0.1f*SCALING );
  tangoCollisionShape_ = new btBoxShape(tangoBoxHalfExtents);
  //collisionShapes_.push_back(tangoCollisionShape_);
  btTransform tangoTransform;
  tangoTransform.setIdentity();
  //tangoTransform.setOrigin(btVector3(1.5f,1.0f,0.0f));
  tangoTransform.setOrigin(tangoOrigin);
  btScalar tangoMass = 0.1f;

  //create the tango rigid body
  //tangoCollisionShape_->setMargin(0.001);
  tangoRigidBody_ = createRigidBody(tangoCollisionShape_,tangoMass,tangoTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(tangoRigidBody_);

  //--------------END OBJECT CREATION SECTION-----------------//

  //--------------CONSTRAINT CREATION SECTION-----------------//
  
  // create a constraint
  //const btVector3 pivotInA( 1.5f, 0.0f, 0.0f );
  const btVector3 pivotInA( 0.0f, 0.0f, 0.0f );   
  const btVector3 pivotInB( 0.0f, 0.0f, 0.0f );
  btVector3 axisInA( 0.0f, 1.0f, 0.0f );
  btVector3 axisInB( 0.0f, 1.0f, 0.0f );
  bool useReferenceFrameA = false;
  hingeConstraint_ = new btHingeConstraint(*bravoRigidBody_,*tangoRigidBody_,pivotInA,pivotInB,axisInA,axisInB,useReferenceFrameA);
  
  // set joint feedback
  hingeConstraint_->setJointFeedback(&jfRobot_);
  // add constraint to the world
  const bool isDisableCollisionsBetweenLinkedBodies = true; //this used to be false
  dynamicsWorld_->addConstraint(hingeConstraint_, 
				  isDisableCollisionsBetweenLinkedBodies);
  
  //--------------END CONSTRAINT CREATION SECTION-----------------//

  //--------------INITIALIZE REMAINING PARAMETERS SECTION-----------------//
  
  //set controller values
  pGains_.setValue(25.0f,25.0f,25.0f);
  dGains_.setValue(2.0f,2.0f,2.0f);
  
  //initialize
  tangoRigidBody_->getMotionState()->getWorldTransform(tangoBodyTrans_);
  currentPose_ = tangoBodyTrans_.getOrigin();
  desiredPose_ = currentPose_;
  //groundRigidBody_->forceActivationState(DISABLE_DEACTIVATION);
  bravoRigidBody_->forceActivationState(DISABLE_DEACTIVATION);
  tangoRigidBody_->forceActivationState(DISABLE_DEACTIVATION);
  bravoRigidBody_->setSleepingThresholds(0.0,0.0);
  tangoRigidBody_->setSleepingThresholds(0.0,0.0);
  //std::cout << "Activation state: " << std::endl;
  //std::cout << tangoRigidBody_->getActivationState() << std::endl;
  count_ = 0;
  
  //--------------END INITIALIZE REMAINING PARAMETERS SECTION-----------------//

  //change friction stuff around
  groundRigidBody_->setFriction(0.3);
  bravoRigidBody_->setFriction(0.3);
  tangoRigidBody_->setFriction(0.3);

  //groundCollisionShape_->setMargin(0.004);
  //bravoCollisionShape_->setMargin(0.004);
  //tangoCollisionShape_->setMargin(0.001);

  if (debug_print_){
    //some debug prints
    btVector3 GaFrict = groundRigidBody_->getAnisotropicFriction();
    btVector3 BaFrict = bravoRigidBody_->getAnisotropicFriction();
    btVector3 TaFrict = tangoRigidBody_->getAnisotropicFriction();
    
    btScalar GFrict = groundRigidBody_->getFriction();
    btScalar BFrict = bravoRigidBody_->getFriction();
    btScalar TFrict = tangoRigidBody_->getFriction();
    
    btScalar GrFrict = groundRigidBody_->getRollingFriction();
    btScalar BrFrict = bravoRigidBody_->getRollingFriction();
    btScalar TrFrict = tangoRigidBody_->getRollingFriction();

    std::cout << "GaFrict: " << GaFrict[0] << "," << GaFrict[1] << "," << GaFrict[2] << std::endl;  
    std::cout << "BaFrict: " << BaFrict[0] << "," << BaFrict[1] << "," << BaFrict[2] << std::endl;  
    std::cout << "TaFrict: " << TaFrict[0] << "," << TaFrict[1] << "," << TaFrict[2] << std::endl;

    std::cout << "GFrict: " << GFrict << std::endl;
    std::cout << "BFrict: " << BFrict << std::endl;
    std::cout << "TFrict: " << TFrict << std::endl;
    
    std::cout << "GrFrict: " << GrFrict << std::endl;
    std::cout << "BrFrict: " << BrFrict << std::endl;
    std::cout << "TrFrict: " << TrFrict << std::endl;

    std::cout << "GMargin: " << groundCollisionShape_->getMargin() << std::endl;
    std::cout << "BMargin: " << bravoCollisionShape_->getMargin() << std::endl;
    std::cout << "TMargin: " << tangoCollisionShape_->getMargin() << std::endl;
  }
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
  startPose_.setValue(prevState[0]*SCALING,1.0*SCALING,prevState[1]*SCALING);
  initPhysics();
  desiredPose_.setValue(action[0]*SCALING,1.0*SCALING,action[1]*SCALING);

  std::cout << "start pose: " << std::endl;
  std::cout << startPose_[0] << std::endl;
  std::cout << startPose_[1] << std::endl;
  std::cout << startPose_[2] << std::endl;  

  std::cout << "desired pose: " << std::endl;
  std::cout << desiredPose_[0] << std::endl;
  std::cout << desiredPose_[1] << std::endl;
  std::cout << desiredPose_[2] << std::endl;

  for(size_t i = 0; i<1000;i++){
    stepWorld();
  }

  std::vector<double> nextState;
  nextState.push_back(bravoBodyTrans_.getOrigin().getX());
  nextState.push_back(bravoBodyTrans_.getOrigin().getZ());

  std::cout << "Simulation is giving me this: " << std::endl;
  std::cout << bravoBodyTrans_.getOrigin().getX() << std::endl;
  std::cout << bravoBodyTrans_.getOrigin().getY() << std::endl;
  std::cout << bravoBodyTrans_.getOrigin().getZ() << std::endl;


  return nextState;
}

/*
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
  
  std::cout << "I made it TO THE END" << std::endl;

  return 0;
}
*/

