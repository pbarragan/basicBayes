//this one is in basicBayesV3wComm

#include <iostream>
#include "BasicDemoPris.h"
//#include <btBulletDynamicsCommon.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>

//Creates a rigid body I think
btRigidBody* BasicDemoPris::createRigidBody( btCollisionShape* collisionShape, 
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
  //std::cout << "I created a rigid body" << std::endl;
  return rigidBody;
}

void BasicDemoPris::pdController(){
  
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

void BasicDemoPris::pController(){
  
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

void BasicDemoPris::stepWorld(){
    
  pdController();

  dynamicsWorld_->stepSimulation(1/60.f,10);
  bravoRigidBody_->getMotionState()->getWorldTransform(bravoBodyTrans_);
  
  if(debug_print_){
    std::cout << "hand X position: " << currentPose_.getX() << std::endl;
    std::cout << "hand Y position: " << currentPose_.getY() << std::endl;
    std::cout << "hand Z position: " << currentPose_.getZ() << std::endl;
    
    
    std::cout << "box X position: " << bravoBodyTrans_.getOrigin().getX() << std::endl;
    std::cout << "box Y position: " << bravoBodyTrans_.getOrigin().getY() << std::endl;
    std::cout << "box Z position: " << bravoBodyTrans_.getOrigin().getZ() << std::endl;
  }
  

}

void BasicDemoPris::initPhysics(){

  //printing
  debug_print_ = false;

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
  groundTransform.setOrigin(btVector3(0.0f,-55.0f,0.0f));
  btScalar groundMass(0.0);

  //create the ground plane rigid body
  groundRigidBody_ = createRigidBody(groundCollisionShape_,groundMass,groundTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(groundRigidBody_);

  //***********************FIGURE THIS OUT*****************//
  //Calculate the required origins based on startPose_ given by the runSimulation function
  /*
  btVector3 tangoOrigin = startPose_;
  btVector3 bravoOrigin = tangoOrigin;
  bravoOrigin.setX(bravoOrigin.getX()-4.5);
  btVector3 alphaOrigin = bravoOrigin;
  alphaOrigin.setX(alphaOrigin.getX()-5.0);
  */

  btVector3 tangoOrigin = startPoseTango_;
  btVector3 bravoOrigin = startPoseBravo_;
  btVector3 alphaOrigin = startPoseAlpha_;
  
  //***********************END FIGURE THIS OUT*****************//

  //make alpha rigid body (the fixed sliding point)
  const btVector3 alphaBoxHalfExtents( 0.5f, 0.5f, 0.5f );
  alphaCollisionShape_ = new btBoxShape(alphaBoxHalfExtents);
  //collisionShapes_.push_back(alphaCollisionShape_);
  btTransform alphaTransform;
  alphaTransform.setIdentity();
  //alphaTransform.setOrigin(btVector3(0.0f,1.0f,0.0f));
  alphaTransform.setOrigin(alphaOrigin);
  btScalar alphaMass = 0.0f;

  //create the bravo rigid body
  alphaRigidBody_ = createRigidBody(alphaCollisionShape_,alphaMass,alphaTransform);
  //add it to the dynamics world
  dynamicsWorld_->addRigidBody(alphaRigidBody_);

  //make bravo rigid body (the sliding beam)
  const btVector3 bravoBoxHalfExtents( 4.0f, 0.5f, 0.5f );
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

  // create a constraint between fixed ground point and sliding beam
  //const btVector3 pivotInA( 1.5f, 0.0f, 0.0f );
  btTransform frameInA = btTransform::getIdentity();
  btTransform frameInB = btTransform::getIdentity();
  bool useReferenceFrameA = true;
  sliderConstraint_ = new btSliderConstraint(*alphaRigidBody_,*bravoRigidBody_, frameInA,frameInB,useReferenceFrameA);
  
  const bool isDisableCollisionsBetweenLinkedBodies = false; //this used to be false
  dynamicsWorld_->addConstraint(sliderConstraint_,isDisableCollisionsBetweenLinkedBodies);
  
  // create a constraint between sliding beam and robot hand
  //const btVector3 pivotInA( 1.5f, 0.0f, 0.0f );
  const btVector3 pivotInA2( 4.5f, 0.0f, 0.0f  );   
  const btVector3 pivotInB2( 0.0f, 0.0f, 0.0f );
  btVector3 axisInA2( 0.0f, 1.0f, 0.0f );
  btVector3 axisInB2( 0.0f, 1.0f, 0.0f );
  bool useReferenceFrameA2 = false;
  hingeConstraint2_ = new btHingeConstraint(*bravoRigidBody_,*tangoRigidBody_,pivotInA2,pivotInB2,axisInA2,axisInB2,useReferenceFrameA2);
  
  // set joint feedback
  hingeConstraint2_->setJointFeedback(&jfRobot_);
  // add constraint to the world
  const bool isDisableCollisionsBetweenLinkedBodies2 = false; //this used to be false
  dynamicsWorld_->addConstraint(hingeConstraint2_, 
				  isDisableCollisionsBetweenLinkedBodies2);
  
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

void BasicDemoPris::exitPhysics(){

  dynamicsWorld_->removeRigidBody(groundRigidBody_);
  delete groundRigidBody_->getMotionState();
  delete groundRigidBody_;
 
  dynamicsWorld_->removeRigidBody(alphaRigidBody_);
  delete alphaRigidBody_->getMotionState();
  delete alphaRigidBody_;
 
  dynamicsWorld_->removeRigidBody(bravoRigidBody_);
  delete bravoRigidBody_->getMotionState();
  delete bravoRigidBody_;

  dynamicsWorld_->removeRigidBody(tangoRigidBody_);
  delete tangoRigidBody_->getMotionState();
  delete tangoRigidBody_;
  
  
  delete groundCollisionShape_;
  delete alphaCollisionShape_;
  delete bravoCollisionShape_;
  delete tangoCollisionShape_;

  delete dynamicsWorld_;

  delete solver_;
  delete collisionConfiguration_;
  delete dispatcher_;
  delete broadphase_;
}

std::vector<double> BasicDemoPris::runSimulation(std::vector<double> prevState, std::vector<double> action){

  //convert the state representation from the basicBayes side into what can be used for the simulation (x,y,z)
  //in the case of the prismatic joint, just pretend it always moves in the x direction and passes through the origin

  //***********************FIGURE THIS OUT*****************//
  //Calculate the required origins based on startPose_ given by the runSimulation function
  startPoseTango_.setValue(prevState[0],1.0,0.0);
  startPoseBravo_.setValue(prevState[0]-4.5,1.0,0.0);
  startPoseAlpha_.setValue(prevState[0]-9.5,1.0,0.0);
  //***********************END FIGURE THIS OUT*****************//


  //startPose_.setValue(prevState[0],1.0,0.0);
  initPhysics();
  desiredPose_.setValue(action[0],1.0,action[1]);

  if (debug_print_){
    std::cout << "start pose: " << std::endl;
    std::cout << startPoseTango_[0] << std::endl;
    std::cout << startPoseTango_[1] << std::endl;
    std::cout << startPoseTango_[2] << std::endl;
    
    std::cout << "desired pose: " << std::endl;
    std::cout << desiredPose_[0] << std::endl;
    std::cout << desiredPose_[1] << std::endl;
    std::cout << desiredPose_[2] << std::endl;
  }

  for(size_t i = 0; i<1000;i++){
    stepWorld();
  }

  std::vector<double> nextState;
  nextState.push_back(tangoBodyTrans_.getOrigin().getX());
  nextState.push_back(tangoBodyTrans_.getOrigin().getZ());

  if (debug_print_){
  std::cout << "Simulation is giving me this: " << std::endl;
  std::cout << tangoBodyTrans_.getOrigin().getX() << std::endl;
  std::cout << tangoBodyTrans_.getOrigin().getY() << std::endl;
  std::cout << tangoBodyTrans_.getOrigin().getZ() << std::endl;
  }

  return nextState;
}

/*
int main (void)
{
 
  BasicDemoPris world;
  
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

