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


//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.

#include "BasicDemo.h"
//#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include <iostream>
#include <fstream>


//#include "GLDebugDrawer.h"

#define _USE_MATH_DEFINES
#include <math.h>

//static GLDebugDrawer gDebugDraw;

void BasicDemo::updateDesiredPose(){
  //this is only 2D for now

  //Update the move angle
  constraintAngle = atan2(appliedImpulse.z(),appliedImpulse.x());

  //mod everything with 2*pi
  constraintAngle = fmod(constraintAngle,2.0*M_PI);
  currentMoveAngle = fmod(currentMoveAngle,2.0*M_PI);
  btScalar tempAngle = constraintAngle+0.5*M_PI;
  if(tempAngle >= currentMoveAngle){
    if(fabs(constraintAngle+0.5*M_PI-currentMoveAngle)<=0.5*M_PI){
      currentMoveAngle = tempAngle;
    }
    else{
      currentMoveAngle = constraintAngle-0.5*M_PI;
    }
  }
  else{
    if(fabs(currentMoveAngle-tempAngle)<=0.5*M_PI){
      currentMoveAngle = tempAngle;
    }
    else{
      currentMoveAngle = constraintAngle-0.5*M_PI;
    }
  }
  btScalar newX = currentPose.x()+moveDist*cos(currentMoveAngle);
  btScalar newZ = currentPose.z()+moveDist*sin(currentMoveAngle);
  desiredPose.setValue(newX,1.0f,newZ);
}

void BasicDemo::thetaToCart(){
  desiredPose.setValue((float)(doorLen*cos(targetAngle)),5.0f,(float)(doorLen*sin(targetAngle)));
  targetAngle += deltaTheta;
}

void BasicDemo::pdController(){
  
  appliedImpulse = hingeConstraint2->getJointFeedback()->m_appliedForceBodyB;
  std::cout << "x force " << appliedImpulse.x() << std::endl;
  std::cout << "y force " << appliedImpulse.y() << std::endl;
  std::cout << "z force " << appliedImpulse.z() << std::endl;

  //get position
  tangoRigidBody->getMotionState()->getWorldTransform(tangoBodyTrans);
  currentPose = tangoBodyTrans.getOrigin();
  //get velocity
  currentVel = tangoRigidBody->getLinearVelocity();

  tangoForce = pGains*(desiredPose - tangoBodyTrans.getOrigin()) - dGains*currentVel;
  tangoRigidBody->applyCentralForce( tangoForce );

  //Write that jazz, man
  if (myfile.is_open()){
    myfile << "tango block x:\t" << tangoBodyTrans.getOrigin().getX() << "\t";
    myfile << "tango block y:\t" << tangoBodyTrans.getOrigin().getY() << "\t";
    myfile << "tango block z:\t" << tangoBodyTrans.getOrigin().getZ() << "\t";

    myfile << "desired door angle:\t" << targetAngle << "\t";
    myfile << "door angle:\t" << atan2(tangoBodyTrans.getOrigin().getZ(),tangoBodyTrans.getOrigin().getX()) << "\t";
    myfile << "x force:\t" << appliedImpulse.x() << "\t";
    myfile << "y force:\t" << appliedImpulse.y() << "\t";
    myfile << "z force:\t" << appliedImpulse.z() << std::endl;
  }
  
}

void BasicDemo::pController(){
  
  appliedImpulse = hingeConstraint2->getJointFeedback()->m_appliedForceBodyB;
  std::cout << "x force " << appliedImpulse.x() << std::endl;
  std::cout << "y force " << appliedImpulse.y() << std::endl;
  std::cout << "z force " << appliedImpulse.z() << std::endl;

  //get position
  tangoRigidBody->getMotionState()->getWorldTransform(tangoBodyTrans);
  currentPose = tangoBodyTrans.getOrigin();
  tangoForce = pGains*(desiredPose - tangoBodyTrans.getOrigin());
  tangoRigidBody->applyCentralForce( tangoForce );

  //Write that jazz, man
  if (myfile.is_open()){
    myfile << "tango block x:\t" << tangoBodyTrans.getOrigin().getX() << "\t";
    myfile << "tango block y:\t" << tangoBodyTrans.getOrigin().getY() << "\t";
    myfile << "tango block z:\t" << tangoBodyTrans.getOrigin().getZ() << "\t";

    myfile << "desired door angle:\t" << targetAngle << "\t";
    myfile << "door angle:\t" << atan2(tangoBodyTrans.getOrigin().getZ(),tangoBodyTrans.getOrigin().getX()) << "\t";
    myfile << "x force:\t" << appliedImpulse.x() << "\t";
    myfile << "y force:\t" << appliedImpulse.y() << "\t";
    myfile << "z force:\t" << appliedImpulse.z() << std::endl;
  }
  
}

void BasicDemo::clientMoveAndDisplay()
{
  	std::cout << "clientMoveAndDisplay" << std::endl;

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	//float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
	  std::cout << "there's a dynamics world" << std::endl;

	  //old way
	  //thetaToCart();
	  //pController();
	  
	  //new way
	  //skip some
	  if (count == 10000){
	    updateDesiredPose();
	    count = 0;
	    std::cout << "NEW POSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	  }
	  else {
	    count +=1;
	  }
	  std::cout << "pre control" << std::endl;

	  pdController();

	  std::cout << "pre step" << std::endl;

	  m_dynamicsWorld->stepSimulation(1/60.f);
	  //optional but useful: debug drawing

    	  std::cout << "post step" << std::endl;


	  std::cout << "desired door angle: " << targetAngle << std::endl;
	  std::cout << "door angle: " << atan2(tangoBodyTrans.getOrigin().getZ(),tangoBodyTrans.getOrigin().getX()) << std::endl;
 
	  std::cout << "activation state: " << bravoRigidBody->getActivationState() << std::endl;
	  
	  //m_dynamicsWorld->debugDrawWorld();
	}
		
	//renderme(); 

	//glFlush();

	//swapBuffers();

}


/*
void BasicDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}
*/

//------------------------------------------------------------------------ADDED
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
//------------------------------------------------------------------------ADDED


void	BasicDemo::initPhysics()
{
  std::cout << "init physics" << std::endl;
  //setTexturing(true);
  //setShadows(true);

  //setCameraDistance(btScalar(SCALING*20.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
	
	//--------------------------------------------------ADDED------------------------------------------------------------------------------
	
	
	// create collision shapes
	//const btVector3 alphaBoxHalfExtents( 0.5f, 0.5f, 0.5f );
	//alphaCollisionShape = new btBoxShape( alphaBoxHalfExtents );
	
	//m_collisionShapes.push_back(alphaCollisionShape);
	
	//
	const btVector3 bravoBoxHalfExtents( 1.0f, 1.0f, 1.0f );
	bravoCollisionShape = new btBoxShape( bravoBoxHalfExtents );
	
	m_collisionShapes.push_back(bravoCollisionShape);
	
	// create alpha rigid body
	//const btScalar alphaMass = 10.0f;
	//const btScalar alphaMass = 0.0f;
	//btTransform alphaTransform;
	//alphaTransform.setIdentity();
	//const btVector3 alphaOrigin( 54.0f, 0.5f, 50.0f );
	//const btVector3 alphaOrigin( -1.5f, 5.0f, 0.0f );
	//alphaTransform.setOrigin( alphaOrigin );
	//alphaRigidBody = createRigidBody( alphaCollisionShape, alphaMass, alphaTransform );
	//m_dynamicsWorld->addRigidBody( alphaRigidBody );
	
	// create bravo rigid body
	const btScalar bravoMass = 1.0f;
	btTransform bravoTransform;
	bravoTransform.setIdentity();
	//const btVector3 bravoOrigin( 56.0f, 0.5f, 50.0f );  
	const btVector3 bravoOrigin( 0.0f, 1.0f, 0.0f );
	bravoTransform.setOrigin( bravoOrigin );
	bravoRigidBody = createRigidBody( bravoCollisionShape, bravoMass, bravoTransform );
	m_dynamicsWorld->addRigidBody( bravoRigidBody );
	  


	// create a constraint
	//btTransform frameInA = btTransform::getIdentity();
	//btTransform frameInB = btTransform::getIdentity();
	
	//std::cout << "slider constraint" << std::endl;
	
	//sliderConstraint = new btSliderConstraint(*alphaRigidBody,*bravoRigidBody, frameInA,frameInB,true);
	//m_dynamicsWorld->addConstraint(sliderConstraint, true);
	

	/*					  
	const btVector3 pivotInA( 1.5f, 0.0f, 0.0f );   
	const btVector3 pivotInB( -4.0f, 0.0f, 0.0f );
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
	hingeConstraint->setJointFeedback(&fg);
	 add constraint to the world
	const bool isDisableCollisionsBetweenLinkedBodies = false;
	m_dynamicsWorld->addConstraint( hingeConstraint, 
					isDisableCollisionsBetweenLinkedBodies );
	*/

	//add a third rigid body to push around
	
	// create collision shapes
	const btVector3 tangoBoxHalfExtents( 0.1f, 0.5f, 0.1f );
	tangoCollisionShape = new btBoxShape( tangoBoxHalfExtents );
	
	m_collisionShapes.push_back(tangoCollisionShape);
	
	// create tango rigid body
	const btScalar tangoMass = 0.1f;
	btTransform tangoTransform;
	tangoTransform.setIdentity();
	const btVector3 tangoOrigin( 1.5f, 1.0f, 0.0f );
	tangoTransform.setOrigin( tangoOrigin );
	tangoRigidBody = createRigidBody( tangoCollisionShape, tangoMass, tangoTransform );
	m_dynamicsWorld->addRigidBody( tangoRigidBody );
	
	//Add another constraint
	
	// create a constraint
	const btVector3 pivotInA2( 1.5f, 0.0f, 0.0f );   
	const btVector3 pivotInB2( 0.0f, 0.0f, 0.0f );
	btVector3 axisInA2( 0.0f, 1.0f, 0.0f );
	btVector3 axisInB2( 0.0f, 1.0f, 0.0f );
	bool useReferenceFrameA2 = false;
	hingeConstraint2 = new btHingeConstraint( 
						 *bravoRigidBody,
						 *tangoRigidBody,
						 pivotInA2,
						 pivotInB2,
						 axisInA2,
						 axisInB2,
						 useReferenceFrameA2);
	
	// set joint feedback
	hingeConstraint2->setJointFeedback(&fg2);
	// add constraint to the world
	const bool isDisableCollisionsBetweenLinkedBodies2 = false;
	m_dynamicsWorld->addConstraint( hingeConstraint2, 
					isDisableCollisionsBetweenLinkedBodies2 );
	
	
	//set controller values
	pGains.setValue(5.0f,5.0f,5.0f);
	dGains.setValue(2.0f,2.0f,2.0f);
	targetAngle = 0.0f;
	deltaTheta = 0.0001f;
	doorLen = 8.5;
	
	//initialize
	tangoRigidBody->getMotionState()->getWorldTransform(tangoBodyTrans);
	currentPose = tangoBodyTrans.getOrigin();
	desiredPose = currentPose;
	bravoRigidBody->forceActivationState(4);
	count = 0;
	moveDist = 0.5;

	srand((unsigned)time(NULL));
	double f = (double)rand() / RAND_MAX;
	currentMoveAngle = ((double)rand() / RAND_MAX) * (2*M_PI);
	//currentMoveAngle = 3.0*M_PI/4.0;
	
	//get the file party started
	//myfile.open("example.txt");
	
	
	//--------------------------------------------------ADDED------------------------------------------------------------------------------
	std::cout << "finished init" << std::endl;

}
	

void	BasicDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	

void	BasicDemo::exitPhysics()
{

  std::cout << "Exiting physics" << std::endl;
  //cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
  std::cout << "Spot 1" << std::endl;

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

  std::cout << "Spot 2" << std::endl;

	m_collisionShapes.clear();
  std::cout << "Spot 2.1" << std::endl;

	delete m_dynamicsWorld;
	  std::cout << "Spot 2.2" << std::endl;

	delete m_solver;
	  std::cout << "Spot 2.3" << std::endl;


	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;
  std::cout << "Spot 3" << std::endl;

	if (myfile.is_open()){
	  myfile.close();
	}

	std::cout << "Done cleaning up" << std::endl;
	
}

int main(){
  BasicDemo world;
  world.initPhysics();
  world.exitPhysics();
  return 0;
}


