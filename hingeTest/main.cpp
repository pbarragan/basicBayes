

#include <iostream>
#include <fstream>

#include <btBulletDynamicsCommon.h>



//code from: http://stackoverflow.com/questions/5998049/physical-simulation-of-realistic-two-legged-skeleton

btScalar targetAngle;

btCollisionShape* alphaCollisionShape;
btCollisionShape* bravoCollisionShape;
btRigidBody* alphaRigidBody;
btRigidBody* bravoRigidBody;
btHingeConstraint* hingeConstraint;

btDynamicsWorld* dynamicsWorld;


//Creates a rigid body I think
btRigidBody* createRigidBody( btCollisionShape* collisionShape, 
			      btScalar mass, 
			      const btTransform& transform )
{
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

//Initialization function 
void Init( btDynamicsWorld* dynamicsWorld )
{
  //this->targetAngle = 0.0f;
  targetAngle = 0.0f;
  //this->dynamicsWorld = dynamicsWorld;
  
  // create collision shapes
  const btVector3 alphaBoxHalfExtents( 0.5f, 0.5f, 0.5f );
  alphaCollisionShape = new btBoxShape( alphaBoxHalfExtents );
  
  //
  const btVector3 bravoBoxHalfExtents( 0.5f, 0.5f, 0.5f );
  bravoCollisionShape = new btBoxShape( bravoBoxHalfExtents );
  
  // create alpha rigid body
  const btScalar alphaMass = 10.0f;
  btTransform alphaTransform;
  alphaTransform.setIdentity();
  const btVector3 alphaOrigin( 54.0f, 0.5f, 50.0f );  
  alphaTransform.setOrigin( alphaOrigin );
  alphaRigidBody = createRigidBody( alphaCollisionShape, alphaMass, alphaTransform );
  dynamicsWorld->addRigidBody( alphaRigidBody );
  
  // create bravo rigid body
  const btScalar bravoMass = 1.0f;
  btTransform bravoTransform;
  bravoTransform.setIdentity();
  const btVector3 bravoOrigin( 56.0f, 0.5f, 50.0f );  
  bravoTransform.setOrigin( bravoOrigin );
  bravoRigidBody = createRigidBody( bravoCollisionShape, bravoMass, bravoTransform );
  dynamicsWorld->addRigidBody( bravoRigidBody );
  
  // create a constraint
  const btVector3 pivotInA( 1.0f, 0.0f, 0.0f );   
  const btVector3 pivotInB( -1.0f, 0.0f, 0.0f );
  btVector3 axisInA( 0.0f, 1.0f, 0.0f );
  btVector3 axisInB( 0.0f, 1.0f, 0.0f );
  bool useReferenceFrameA = false;
  hingeConstraint = new btHingeConstraint( 
					  *alphaRigidBody,
					  *bravoRigidBody,
					  pivotInA,
					  pivotInB,
					  axisInA,
					  axisInB,
					  useReferenceFrameA );
  
  // set constraint limit
  const btScalar low = -M_PI;
  const btScalar high = M_PI;
  hingeConstraint->setLimit( low, high );
  
  // add constraint to the world
  const bool isDisableCollisionsBetweenLinkedBodies = false;
  dynamicsWorld->addConstraint( hingeConstraint, 
				isDisableCollisionsBetweenLinkedBodies );
}


//update function using the motor
void Update( float deltaTime )
{
  alphaRigidBody->activate();
  bravoRigidBody->activate();
  
  bool isEnableMotor = true;
  btScalar maxMotorImpulse = 1.0f; // 1.0f / 8.0f is about the minimum
  
  hingeConstraint->enableMotor( isEnableMotor );
  hingeConstraint->setMaxMotorImpulse( maxMotorImpulse );
  
  targetAngle += 0.1f * deltaTime;
  hingeConstraint->setMotorTarget( targetAngle, deltaTime );  
}

//the main function 
int main (void)
{
  //This is always necessary
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();
  
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
  
  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
  
  //dynamicsWorld->setGravity(btVector3(0,-10,0));
  dynamicsWorld->setGravity(btVector3(0,0,0));
  //Do everything else

  
  //initialize the world
  Init(dynamicsWorld);

  float deltaTime = 1/60.f;

  std::ofstream myfile ("example.txt");

  if (myfile.is_open()){
    for (int i=0 ; i<3000 ; i++) {
      dynamicsWorld->stepSimulation(deltaTime,10);
      
      btTransform transA;
      alphaRigidBody->getMotionState()->getWorldTransform(transA);

      btTransform transB;
      bravoRigidBody->getMotionState()->getWorldTransform(transB);
      
      //std::cout << "2nd block x: \t" << trans.getOrigin().getX() << std::endl;
      //std::cout << "2nd block y: \t" << trans.getOrigin().getY() << std::endl;
      //std::cout << "2nd block z: \t" << trans.getOrigin().getZ() << std::endl;
      myfile << "1st block x:\t" << transA.getOrigin().getX() << "\t";
      myfile << "2nd block x:\t" << transB.getOrigin().getX() << std::endl;
      myfile << "1st block y:\t" << transA.getOrigin().getY() << "\t";
      myfile << "2nd block y:\t" << transB.getOrigin().getY() << std::endl;
      myfile << "1st block z:\t" << transA.getOrigin().getZ() << "\t";
      myfile << "2nd block z:\t" << transB.getOrigin().getZ() << std::endl;
      Update(deltaTime);
    }
    myfile.close();
  }
  
  else std::cout << "Unable to open file" << std::endl;
  
  //this is old stuff
  /*btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  
  btCollisionShape* fallShape = new btSphereShape(1);
  
  
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
  btRigidBody::btRigidBodyConstructionInfo
    groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody);
  
  
  btDefaultMotionState* fallMotionState =
    new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0)));
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  dynamicsWorld->addRigidBody(fallRigidBody);
  
  
  for (int i=0 ; i<300 ; i++) {
    dynamicsWorld->stepSimulation(1/60.f,10);
    
    btTransform trans;
    fallRigidBody->getMotionState()->getWorldTransform(trans);
    
    std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
  }
  */

  //this is cleanup
  //old stuff
  /*dynamicsWorld->removeRigidBody(fallRigidBody);
  delete fallRigidBody->getMotionState();
  delete fallRigidBody;
  
  dynamicsWorld->removeRigidBody(groundRigidBody);
  delete groundRigidBody->getMotionState();
  delete groundRigidBody;
  
  
  delete fallShape;
  
  delete groundShape;
  */

  //This always needs to be here
  delete dynamicsWorld;
  delete solver;
  delete collisionConfiguration;
  delete dispatcher;
  delete broadphase;
  
  return 0;
}

