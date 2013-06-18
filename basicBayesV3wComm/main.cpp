//this is in basicBayesV3wComm

#include <iostream>
#include <fstream>
//#include "basicBayesV1.h"
//#include "eigenMultivariateNormal.h"
#include "Eigen/Dense"

#define _USE_MATH_DEFINES
#include <math.h>
using Eigen::MatrixXd;
//#include <bullshit>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "basicBayes.h"
#include "BasicDemo.h"
#include "BasicDemoRev.h"
#include "BasicDemoPris.h"
#include "bbSide.h"

void testPrint(){
  std::cout << "apparently this works" << std::endl;
}

double vectorDiff(std::vector<double> a, std::vector<double> b){
  double sum = 0;
  for (size_t i=0;i<a.size();i++){
    sum += (a[i]-b[i])*(a[i]-b[i]);
  }
  return sum;
}

std::vector<double> basicBayes::normalizeVector(std::vector<double> probVect){
	double sum = std::accumulate(probVect.begin(),probVect.end(),(double) 0);
	for (size_t i = 0; i<probVect.size(); i++) {
		probVect[i] /= sum; 
	}
	return probVect;
}

std::vector<double> basicBayes::stateTransition(std::vector<double> prevState, std::vector<double> action){	
	//this funciton is problem specific

	//make this do something eventually
	std::vector<double> nextState;
	double sum = std::accumulate(action.begin(),action.end(),(double) 0);
	if (sum == -100.0) {
	  //Didn't do anything kind of this has to change
	  nextState = prevState;
	}
	else {
	  //this is saying there is no probability to transition between models
	  //prevState[0] is the model. 0.0 is free, 1.0 is fixed, 2.0 is revolute, 3.0 is prismatic

	  //WOM means without model.
	  std::vector<double> prevStateWOM;
	  
	  prevStateWOM.assign(prevState.begin()+1,prevState.end()); //state without the model which is always the first entry in the state

	  std::vector<double> tempWorldState;

	  if (prevState[0]==0.0) {
	    //model: free
	    BasicDemo world;
	    tempWorldState = world.runSimulation(prevStateWOM,action);
	    nextState.push_back(0.0); //this is saying you end up in the same state
	    for(size_t j=0;j<tempWorldState.size();j++){
	      nextState.push_back(tempWorldState[j]);
	    }
	  }
	  else if (prevState[0]==1.0) {
	    //model: fixed
	    nextState=prevState;
	  }
	  else if (prevState[0]==2.0) {
	    //model: revolute
	    BasicDemoRev world;
	    tempWorldState = world.runSimulation(prevStateWOM,action);
	    nextState.push_back(2.0); //this is saying you end up in the same state
	    for(size_t j=0;j<tempWorldState.size();j++){
	      nextState.push_back(tempWorldState[j]);
	    }
	  }
	  else if (prevState[0]==3.0) {
	    //model: prismatic
	    BasicDemoPris world;
	    tempWorldState = world.runSimulation(prevStateWOM,action);
	    nextState.push_back(3.0); //this is saying you end up in the same state
	    for(size_t j=0;j<tempWorldState.size();j++){
	      nextState.push_back(tempWorldState[j]);
	    }
	  }

	  //print out what the simulation returned each time
	  if (debug_print_){
	    std::cout << "the simulation returned:" << std::endl;
	    for(size_t j=0;j<tempWorldState.size();j++){
	      std::cout << tempWorldState[j] << "," << std::endl;
	    }		
	  }

	}
	
	return nextState;
}

//overload this function
void basicBayes::transitionUpdate(std::vector<double> action){
  transitionUpdate(probList_, action);
  //printProbList();
}

//overload this function
void basicBayes::transitionUpdate(std::vector<double>& probList, std::vector<double> action){
	//in here, we need to for each state 1) do a state transition 2) drop a guassian on that as the mean, then for each state
	//3) calculate the probility of that state in that gaussian 4) multiple it by the prev probability of the state in the outer for loop
	//5) sum the probability added to each state as you go through the outer for loop
	//this will require a termporary probability list.

  //print out the action taken every time the transition update is called
  if (debug_print_){
    std::cout << "Took an action: " << std::endl;
    for (size_t ll=0; ll<action.size(); ll++) {
      std::cout << action[ll] << ',';
    }
    std::cout << std::endl;
  }

	std::vector<double> tempProbList (stateList.size(),0.0); //this will be the sum of the probability after each x_k-1 state

	for (size_t i=0; i<stateList.size(); i++) {
		//this loop is for x_k-1
		std::vector<double> tempStateProbList (stateList.size(),0.0); //this will hold the probability in the inner loop waiting for normalization

		std::vector<double> nextState = stateTransition(stateList[i], action); //this will be the mean of the guassian used to calculate the transition probability
		
		for (size_t j=0; j<stateList.size(); j++) {
			//this loop is for x_k
			tempStateProbList[j] = probState(stateList[j],nextState);
			//std::cout << "probState: " << tempStateProbList[j] << std::endl;
		}
		tempStateProbList = normalizeVector(tempStateProbList); //normalize the distribution before you scale and add it.

		for (size_t w=0; w<tempStateProbList.size(); w++){
		  //std::cout << "inner prob: " << tempStateProbList[w] << std::endl;
		}
		
		for (size_t k=0; k<stateList.size(); k++) {
			//this loop is for x_k
			tempProbList[k] += tempStateProbList[k]*probList[i];
		}
	}
	//this is debug stuff
	//double sumBefore = std::accumulate(probList.begin(),probList.end(),(double) 0);
	probList = tempProbList;
	//double sumAfter = std::accumulate(probList.begin(),probList.end(),(double) 0);

	//std::cout << "Sum Before: " << sumBefore << std::endl;
	//std::cout << "Sum After: " << sumAfter << std::endl;
	//printProbList();

}

//overload this function
void basicBayes::observationUpdate(std::vector<double> obs){
  observationUpdate(probList_,obs);
  //printProbList();
}

//overload this function
void basicBayes::observationUpdate(std::vector<double>& probList, std::vector<double> obs){
  if (debug_print_){
    std::cout << "Real current state: " << std::endl;
    std::cout << realCurrentState_[0] << "," << realCurrentState_[1] << std::endl;
    
    std::cout << "Got an observation:" << std::endl;
    
    for (size_t ll=0; ll<obs.size(); ll++) {
      std::cout << obs[ll] << ',';
    }
    std::cout << std::endl;
  }

	for (size_t i=0; i<stateList.size(); i++) {
		probList[i] *= probObs(obs,stateList[i]);
	}
	
	//normalize
	//printProbList();
	//double sumBefore = std::accumulate(probList.begin(),probList.end(),(double) 0);

	probList = normalizeVector(probList);
	//double sumAfter = std::accumulate(probList.begin(),probList.end(),(double) 0);
	//std::cout << "Sum Before: " << sumBefore << std::endl;
	//std::cout << "Sum After: " << sumAfter << std::endl;
	//printProbList();
}
 
double basicBayes::probState(std::vector<double> sampleState, std::vector<double> meanState){
	//this funciton is problem specific
	
	//sampleState is the sample vector. meanState is the mean vector. This just drops a gaussian at the meanState with a constant covariance from the class.
	//return evaluteMVG(sampleState,meanState,transCovMat);
	
	if (meanState[0]==sampleState[0]){
		//WOM means without model.
		std::vector<double> sampleStateWOM;
		std::vector<double> meanStateWOM;
	

		//different things for different states	
		if (sampleState[0] == 0.0 || sampleState[0] == 1.0){
		  sampleStateWOM.assign(sampleState.begin()+1,sampleState.end());
		}
		else if (sampleState[0] == 2.0){
		  double beamLen = 0.44-0.1;
		  double beamAngle = sampleState[1]; //angle of the beam
		  sampleStateWOM.push_back(0.44+(beamLen+0.1f)*cos(beamAngle));
		  sampleStateWOM.push_back((beamLen+0.1f)*sin(beamAngle));
		}
		else if (sampleState[0] == 3.0){
		  double beamSlidePosition = sampleState[1]; //slide position parameter
		  sampleStateWOM.push_back(beamSlidePosition);
		  sampleStateWOM.push_back(0.0);
		}
		
		meanStateWOM.assign(meanState.begin()+1,meanState.end());
	
		//std::cout << "I'm in probState" << std::endl;
		//std::cout << "Model:" << sampleState[0] << std::endl;
		//std::cout << sampleStateWOM[0] << "," << sampleStateWOM[1] << std::endl;
		//std::cout << meanStateWOM[0] << "," << meanStateWOM[1] << std::endl;
		return evaluteMVG(sampleStateWOM,meanStateWOM,transCovMat);
	}
	else {
		return 0.0;
	}

}

double basicBayes::probObs(std::vector<double> obs, std::vector<double> state){
	//this funciton is problem specific

	//obs is the sample vector. state is the mean vector. This just drops a gaussian at the state with a constant covariance from the class.
	//return evaluteMVG(obs,state,obsCovMat);
	
	//WOM means without model.
	std::vector<double> stateWOM;
	
	//different things for different states	
	if (state[0] == 0.0 || state[0] == 1.0){
	  stateWOM.assign(state.begin()+1,state.end());
	}
	else if (state[0] == 2.0){
	  double beamLen = 0.44-0.1;
	  double beamAngle = state[1]; //angle of the beam
	  stateWOM.push_back(0.44+(beamLen+0.1f)*cos(beamAngle));
	  stateWOM.push_back((beamLen+0.1f)*sin(beamAngle));
	}
	else if (state[0] == 3.0){
	  double beamSlidePosition = state[1]; //slide position parameter
	  stateWOM.push_back(beamSlidePosition);
	  stateWOM.push_back(0.0);
	}
	
	//std::cout << "I'm in probState" << std::endl;

	return evaluteMVG(obs,stateWOM,obsCovMat);
}

Eigen::Map<Eigen::MatrixXd> basicBayes::convertVect(std::vector<double>& vect)
{
	Eigen::Map<Eigen::MatrixXd> returnVec(&vect[0],vect.size(),1);
	return returnVec;
}

Eigen::Map<Eigen::MatrixXd> basicBayes::convertCovMat(std::vector<double>& covMat)
{
	Eigen::Map<Eigen::MatrixXd> returnCovMat(&covMat[0],sqrt(covMat.size()),sqrt(covMat.size()));
	return returnCovMat;
}

//Need to set the covariance matrices for the transition function (simulated action) and the observation function
void basicBayes::setCovMats(std::vector<double>& obs,std::vector<double>& trans){
	//this funciton is problem specific
	
	//double obsArray[] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
	//double transArray[] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
	double obsArray[] = {0.005,0.0,0.0,0.005};
	double transArray[] = {0.005,0.0,0.0,0.005};
	obs.assign(obsArray, obsArray + sizeof(obsArray)/sizeof(double));
	trans.assign(transArray, transArray + sizeof(transArray)/sizeof(double));
} 

//create the value list to set up states
std::vector< std::vector<double> > basicBayes::createValueList(double dimRanges[][2], double dimNums[], int dims){
	//there might be a shorter way 
	std::vector< std::vector<double> > valueList;
	for (size_t i=0; i<dims; i++) {
		double delta = (dimRanges[i][1]-dimRanges[i][0])/(dimNums[i]-1);
		std::vector<double> tempVect;
		for (size_t j=0; j<dimNums[i]; j++) {
			tempVect.push_back(dimRanges[i][0]+delta*j);
		}
		valueList.push_back(tempVect);
	}
	return valueList;
}


//the recursive function
std::vector< std::vector<double> > basicBayes::recurseList(std::vector< std::vector<double> > totalList, std::vector<double> oldSeq, int level, std::vector< std::vector<double> > valueList){
	//bottomLevel = valueList.size();
	if (level>valueList.size()-1) {
		totalList.push_back(oldSeq);
	}
	else {
		for (size_t i=0; i<valueList[level].size(); i++) {
			oldSeq.push_back(valueList[level][i]);
			totalList = recurseList(totalList,oldSeq,level+1,valueList);
			oldSeq.pop_back();
		}
	}
	return totalList;
}

//Need to create the list of actions
void basicBayes::setupActions(){
  //the format of this is that dimRanges (dimension ranges) gives you the min and max in each dimension.
  //dimNums gives you the number of discrete points along a dimension.
  
  double dimRanges[][2]={{-0.10,0.10},{-0.10,0.10}};
  double dimNums[] = {3,3};
  int dims = sizeof(dimNums)/sizeof(dimNums[0]);
  
  std::vector< std::vector<double> > valueList = createValueList(dimRanges,dimNums,dims);
  std::vector< std::vector<double> > actionList = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueList);

  actionList_ = actionList;
}

//Need to create the list of states
void basicBayes::setupStates(){
	//this funciton is problem specific

	//create valueList try 1
	std::vector< std::vector<double> > valueList;
	std::vector<double> row1;
	std::vector<double> row2;
	std::vector<double> row3;
	row1.push_back(0);
	row1.push_back(1);
	row1.push_back(2);
	row2.push_back(0);
	row2.push_back(1);
	row2.push_back(2);
	row2.push_back(3);
	row3.push_back(0);
	row3.push_back(1);
	valueList.push_back(row1);
	valueList.push_back(row2);
	valueList.push_back(row3);
		
	//try 2
	//the format of this is that dimRanges (dimension ranges) gives you the min and max in each dimension.
	//dimNums gives you the number of discrete points along a dimension.
	/*
	double dimRanges[][2]={{0,1},{0,0.15},{0,0.15}};
	double dimNums[] = {2,2,2};
	int dims = sizeof(dimNums)/sizeof(dimNums[0]);
	
	std::vector< std::vector<double> > valueList2 = createValueList(dimRanges,dimNums,dims);
	stateList = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueList2);
	*/

	//try 3
	//Set up a state list for each model. 
	//Then add the model as the first entry to the appropriate lists. 
	//Then stick together all of the lists into one master list. 
	
	//the format of this is that dimRanges (dimension ranges) gives you the min and max in each dimension.
	//dimNums gives you the number of discrete points along a dimension.

	//model 0 is the free model
	double dimRanges0[][2]={{-0.50,0.50},{-0.50,0.50}};
	double dimNums0[] = {11,11};
	int dims0 = sizeof(dimNums0)/sizeof(dimNums0[0]);
	
	std::vector< std::vector<double> > valueListA = createValueList(dimRanges0,dimNums0,dims0);
	std::vector< std::vector<double> > stateList0 = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueListA);

	for (size_t r = 0; r<stateList0.size(); r++){
	  stateList0[r].insert(stateList0[r].begin(),0.0);
	}

	/*
	for (size_t b = 0; b<valueListA.size(); b++){
	  for (size_t c = 0; c<valueListA[b].size(); c++){
	    std::cout << valueListA[b][c] << ",";
	  }
	  std::cout << std::endl;
	}
	*/

	//model 1 is the fixed model
	double dimRanges1[][2]={{0.0,0.0},{0.0,0.0}};
	double dimNums1[] = {1,1};
	int dims1 = sizeof(dimNums1)/sizeof(dimNums1[0]);
	
	std::vector< std::vector<double> > valueListB = createValueList(dimRanges1,dimNums1,dims1);

	valueListB[0][0] = 0.0;
	valueListB[1][0] = 0.0;

	std::vector< std::vector<double> > stateList1 = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueListB);

	for (size_t s = 0; s<stateList1.size(); s++){
	  stateList1[s].insert(stateList1[s].begin(),1.0);
	}

	/*
	for (size_t b = 0; b<valueListB.size(); b++){
	  for (size_t c = 0; c<valueListB[b].size(); c++){
	    std::cout << valueListB[b][c] << ",";
	  }
	  std::cout << std::endl;
	}
	*/

	//model 2 is the revolute model
	double dimRanges2[][2]={{M_PI/2.0,3.0*M_PI/2.0}};
	double dimNums2[] = {30};
	int dims2 = sizeof(dimNums2)/sizeof(dimNums2[0]);
	
	std::vector< std::vector<double> > valueListC = createValueList(dimRanges2,dimNums2,dims2);
	std::vector< std::vector<double> > stateList2 = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueListC);

	for (size_t s = 0; s<stateList2.size(); s++){
	  stateList2[s].insert(stateList2[s].begin(),2.0);
	}

	//model 3 is the prismatic model
	double dimRanges3[][2]={{-.30,0.30}};
	double dimNums3[] = {30};
	int dims3 = sizeof(dimNums3)/sizeof(dimNums3[0]);
	
	std::vector< std::vector<double> > valueListD = createValueList(dimRanges3,dimNums3,dims3);
	std::vector< std::vector<double> > stateList3 = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueListD);

	for (size_t s = 0; s<stateList3.size(); s++){
	  stateList3[s].insert(stateList3[s].begin(),3.0);
	}

	//stick everything together
	stateList0.insert( stateList0.end(), stateList1.begin(), stateList1.end());
	stateList0.insert( stateList0.end(), stateList2.begin(), stateList2.end());
	stateList0.insert( stateList0.end(), stateList3.begin(), stateList3.end());

	stateList = stateList0;

	
	//do a bunch of stuff here:
	//this section sets up the state probability as a uniform prior over models which is not necessarily equal over all the states.
	
	//old way
	//double tempProb = 1.0/stateList.size();
	//probList.assign(stateList.size(),tempProb);

	//new way
	numModels_ = 4;
	double probPerModel = 1.0/numModels_;
	std::vector<double> tempProbListUniform;

	//model 0
	int prod0 = 1;
	for (size_t jj = 0; jj<dims0; jj++){
		prod0 *= dimNums0[jj];
	}
	tempProbListUniform.push_back(probPerModel/prod0);

	//model 1
	int prod1 = 1;
	for (size_t jj = 0; jj<dims1; jj++){
		prod1 *= dimNums1[jj];
	}
	tempProbListUniform.push_back(probPerModel/prod1);

	//model 2
	int prod2 = 1;
	for (size_t jj = 0; jj<dims2; jj++){
		prod2 *= dimNums2[jj];
	}
	tempProbListUniform.push_back(probPerModel/prod2);

	//model 3
	int prod3 = 1;
	for (size_t jj = 0; jj<dims3; jj++){
		prod3 *= dimNums3[jj];
	}
	tempProbListUniform.push_back(probPerModel/prod3);

	
	for (size_t f = 0; f<stateList.size(); f++){
	  probList_.push_back(tempProbListUniform[(int) stateList[f][0]]);
	}
	
	

	/*
	//This section prints useful things
	int prod = 1;
	for (size_t jj = 0; jj<dims; jj++){
		prod *= dimNums[jj];
	}
	std::cout << "number of states expected: " << prod << std::endl;
	std::cout << "number of states: " << stateList.size() << std::endl;
	*/

	printStateList();
	printProbList();

}

/*
//Version 1
double evaluteMVG(Eigen::Matrix<double,2,1>& sampleVec, Eigen::Matrix<double,2,1>& meanVec, Eigen::Matrix<double,2,2>& covMat){
	Eigen::Matrix<double,2,1> error = (sampleVec - meanVec);
	Eigen::Matrix<double,1,1> secondHalf = (error.transpose()*covMat.inverse()*error);
	return (1.0/(pow(2.0*M_PI,2.0/2.0)*pow(covMat.determinant(),0.5)))*exp(-0.5*secondHalf(0,0));
}
*/

/*
//Version 2
double evaluteMVG(Eigen::MatrixXd& sampleVec, Eigen::MatrixXd& meanVec, Eigen::MatrixXd& covMat){
	Eigen::MatrixXd error = (sampleVec - meanVec);
	Eigen::Matrix<double,1,1> secondHalf = (error.transpose()*covMat.inverse()*error);
	return (1.0/(pow(2.0*M_PI,meanVec.rows()/2.0)*pow(covMat.determinant(),0.5)))*exp(-0.5*secondHalf(0,0));
}
*/

/*
//Version 3
double evaluteMVG(Eigen::Map<Eigen::MatrixXd>& sampleVec, Eigen::Map<Eigen::MatrixXd>& meanVec, Eigen::Map<Eigen::MatrixXd>& covMat){
	Eigen::MatrixXd error = (sampleVec - meanVec);
	Eigen::Matrix<double,1,1> secondHalf = (error.transpose()*covMat.inverse()*error);
	return (1.0/(pow(2.0*M_PI,meanVec.rows()/2.0)*pow(covMat.determinant(),0.5)))*exp(-0.5*secondHalf(0,0));
}
*/

//Version 4
double basicBayes::evaluteMVG(std::vector<double>& sampleVecVect, std::vector<double>& meanVecVect, std::vector<double>& covMatVect){
	//this funciton is problem specific
	//Convert
	Eigen::Map<Eigen::MatrixXd> sampleVec = convertVect(sampleVecVect);
	Eigen::Map<Eigen::MatrixXd> meanVec = convertVect(meanVecVect);
	Eigen::Map<Eigen::MatrixXd> covMat = convertCovMat(covMatVect);
	
	Eigen::MatrixXd error = (sampleVec - meanVec);
	Eigen::Matrix<double,1,1> secondHalf = (error.transpose()*covMat.inverse()*error);
	return (1.0/(pow(2.0*M_PI,meanVec.rows()/2.0)*pow(covMat.determinant(),0.5)))*exp(-0.5*secondHalf(0,0));
}


/*
std::string doubleToString ( double Number )
{
	std::ostringstream ss;
	ss << Number;
	return ss.str();
}
*/

void basicBayes::filterSetup(){
  //seed the random number generator once for the filter.
  srand((unsigned)time(NULL));

  setCovMats(obsCovMat,transCovMat);

}

void basicBayes::printProbList(){
	std::cout << "Printing Probablity List:" << std::endl;
	for (size_t ii = 0; ii<probList_.size(); ii++) {
		std::cout << probList_[ii] <<std::endl;
	}
}

void basicBayes::printStateList(){
	std::cout << "Printing State List:" << std::endl;
	for (size_t ii = 0; ii<stateList.size(); ii++) {
		for (size_t jj = 0; jj<stateList[ii].size(); jj++) {
			std::cout << stateList[ii][jj] << ',';
		}
		std::cout << std::endl;
	}
}

//double basicBayes::modelProbability()

//overload this function
std::vector<double> basicBayes::getObs(){
  return getObs(realCurrentState_);
}

//overload this function
std::vector<double> basicBayes::getObs(std::vector<double>& currentState){
  std::vector<double> stateWNoise;
  for (size_t i=0; i<currentState.size(); i++){
    //srand((unsigned)time(NULL));
    //double X=((double)rand()/(double)RAND_MAX);
    double X=randomDouble();
    stateWNoise.push_back(currentState[i]+0.001*X);
  }
  return stateWNoise;
}

//overload this function
std::vector<double> basicBayes::calcModelProb(){
  return calcModelProb(probList_);
}

//overload this function
std::vector<double> basicBayes::calcModelProb(std::vector<double>& probList){
  
  std::vector<double> sumHold (4,0.0);
  
  for (size_t i=0; i<stateList.size(); i++){
    sumHold[(int) stateList[i][0]] += probList[i];
  }
  
  return sumHold;
  
}

double basicBayes::distPoints(std::vector<double> a, std::vector<double> b){
  //this is meant for 2d distances
  return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]));
}

std::vector<double> basicBayes::createCDF(std::vector<double> probList){
  std::vector<double> probCDF;
  probCDF.push_back(probList[0]); //initialize the first entry for the list
  for (size_t i=1; i<probList.size(); i++){
    probCDF.push_back(probCDF[i-1]+probList[i]);
  }
  return probCDF;
}

std::vector<double> basicBayes::getSampleState(std::vector<double>& CDF, std::vector< std::vector<double> >& states){
  double X = randomDouble();
  std::vector<double>::iterator low = std::lower_bound(CDF.begin(),CDF.end(),X);
  return states[std::distance(CDF.begin(),low)];
}

double basicBayes::calcEntropy(std::vector<double> probs){
  double sum=0;
  for (size_t i=0; i<probs.size(); i++){
    if (probs[i]!=0.0){
      sum += probs[i]*log(probs[i]);
    }
  }
  return -sum;
}

void basicBayes::chooseAction(){
  //The new method samples from the belief state according to the probability distribution and simulates from those states for each action. An observation is taken for the output state after the simulation. Given the observation, the belief state is updated. The probability distribution over models is calculated. The entropy of this distribution is calculated. The action which produces the lowest entropy is chosen. 
  //Step 1: Create the CDF of the current belief from the PDF probList_.
  std::vector<double> probCDF = createCDF(probList_);

  //Step 2: For each action, sample a state from the belief n times. Simulate this state with the action and get an observation. Update the belief with the action-observation pair. Calculate the entropy of the new belief. Average the entropies over the n samples.
  std::vector<double> avgEntropyList; //this is a list of average entropies, one for each action
  int nSamples = 1; //number of samples of the belief state per action

  for (size_t i = 0; i<actionList_.size(); i++){
    std::vector<double> entropyList; //this is per action
    for (size_t j = 0; j<nSamples; j++){

      //Step 2.0: Create a copy of the real probability list
      std::vector<double> localProbList = probList_; //only for this action and sample

      //Step 2.1: Sample a state from the belief
      std::vector<double> sample = getSampleState(probCDF,stateList);
	
      if (debug_print_){
	std::cout << "Choose an action: " << std::endl;
	for (size_t k=0; k<sample.size(); k++){
	  std::cout << sample[k] << ",";
	}
	std::cout << std::endl;
      }
      /*
	filter.runRealWorld();
	
	filter.transitionUpdate(filter.realAction_);
	//if you want noise added to the observation from the simulation
	filter.observationUpdate(filter.getObs());
      */

      //std::vector<double> tempVect = probList_;
      //Step 2.2: Simulate the state with the action
      std::vector<double> nextState = stateTransition(sample, actionList_[i]);
      std::vector<double> nextStateWOM;

      //std::cout << "probList difference: " << vectorDiff(tempVect,probList_) << std::endl;

      nextStateWOM.assign(nextState.begin()+1,nextState.end()); //state without the model which is always the first entry in the state
      //std::cout << "next state:" << nextStateWOM[0] << "," << nextStateWOM[1] << std::endl;

      //Step 2.3: Get an observation
      //Step 2.4: Update the belief state
      transitionUpdate(localProbList,actionList_[i]);
      observationUpdate(localProbList,getObs(nextStateWOM));

      //Step 2.5: Calculate the entropy over models of the new belief state
      std::vector<double> mProbs = calcModelProb(localProbList);
      entropyList.push_back(calcEntropy(mProbs));

    }
    //Step 2.6: Average the entropies over the n samples
    double eSum = std::accumulate(entropyList.begin(),entropyList.end(),(double) 0.0);
    avgEntropyList.push_back(eSum/entropyList.size());
  }

  //Step 3: Choose the action which results in the lowest entropy updated belief
  std::vector<double>::iterator minAvgEntIt = std::min_element(avgEntropyList.begin(),avgEntropyList.end());
  realAction_ = actionList_[std::distance(avgEntropyList.begin(),minAvgEntIt)];

}

void basicBayes::chooseActionOG(){
  //OG == Original Gangster
  //The original method was simply to run a bunch of actions on the top two models in terms of probability and try to determine which action produced the most different outcome and then choose that one.
  //this will only work if there is at least two models
  //and the probabilities better be positive

  //Step 1: Calculate all the model probabilities so you know which models to compare (the highest two)
  std::vector<double> modelProbs = calcModelProb();

  if (modelProbs.size()<2){
    std::cout << "At least 2 models needed" << std::endl;
  }
  else {
    //std::cout << "Let's choose a model" << std::endl;
    double first=-1.0;
    double second=-1.0;
    int firstIndex=-1;
    int secondIndex=-1;
    
    for (size_t i = 0; i<modelProbs.size(); i++){
      if (modelProbs[i]>first){
	second = first;
	secondIndex = firstIndex;
	first = modelProbs[i];
	firstIndex = i;
      }
      else if (modelProbs[i]>second){
	second = modelProbs[i];
	secondIndex = i;
      }
    }
    
    std::cout << "First model: " << firstIndex << std::endl;
    std::cout << "Second model: " << secondIndex << std::endl;
    
    //Step 2: Figure out the maximum probability state for those models to simulate from
    std::vector<double> firstState (1,0.0);
    double firstStateProb = -1.0;
    std::vector<double> secondState (1,0.0);
    double secondStateProb = -1.0;
    
    for (size_t i = 0; i<stateList.size(); i++){
      if ((int) stateList[i][0] == firstIndex){
	if (probList_[i] > firstStateProb){
	  firstState = stateList[i];
	  firstStateProb = probList_[i];
	}
      }
      else if ((int) stateList[i][0] == secondIndex){
	if (probList_[i] > secondStateProb){
	  secondState = stateList[i];
	  secondStateProb = probList_[i];
	}
      }
    }
    
    //Step 3: Simulate all the actions from the states found in Step 2
    std::vector< std::vector<double> > firstNextStates;
    std::vector< std::vector<double> > secondNextStates;
    
    for (size_t i=0; i<actionList_.size(); i++){
      firstNextStates.push_back(stateTransition(firstState,actionList_[i]));
      secondNextStates.push_back(stateTransition(secondState,actionList_[i]));
    }
    
    //Step 4: Calculate distances between results from Step 3 and determine which is greatest
    std::vector<double> stateDistances;
    
    //WOM means without model.
    std::vector<double> firstNextStateWOM;
    std::vector<double> secondNextStateWOM;
    
    double furthestDist = -1.0;
    double currentDist = -2.0;
    std::vector<double> bestAction (2,0.0);
    
    for (size_t i=0; i<actionList_.size(); i++){
      firstNextStateWOM.assign(firstNextStates[i].begin()+1,firstNextStates[i].end()); //state without the model which is always the first entry in the state
      secondNextStateWOM.assign(secondNextStates[i].begin()+1,secondNextStates[i].end()); //state without the model which is always the first entry in the state
      currentDist = distPoints(firstNextStateWOM,secondNextStateWOM);
      stateDistances.push_back(currentDist);
      
      if (currentDist > furthestDist){
	furthestDist = currentDist;
	bestAction = actionList_[i];
      }
    }
    
    //Step 5: Set the next action to do to the best action found
    realAction_ = bestAction;
    
  }
  
}  
void basicBayes::runRealWorld(){

  bool realWorld = true;

  if (realWorld == false){
  //When you want to use a simulation for the real world
  
    int realModel = 3;

    std::cout << "this is the real world" << std::endl;
  
    if (realModel == 0){
      BasicDemo world;
      realCurrentState_ = world.runSimulation(realPrevState_,realAction_);
    }
    else if (realModel == 1){
      realCurrentState_ = realPrevState_;
    }
    else if (realModel == 2){
      BasicDemoRev world;
      std::vector<double> temp;
      temp.push_back(atan2(realPrevState_[1],realPrevState_[0]-0.44));
      realCurrentState_ = world.runSimulation(temp,realAction_);
    }
    else if (realModel == 3){
      BasicDemoPris world;
      std::vector<double> temp;
      temp.push_back(realPrevState_[0]);
      realCurrentState_ = world.runSimulation(temp,realAction_);
    }
    
  }
  
  //When you want to use the robot for the real world
  //translation of the coordinate systems is required
  //bullet: +x forward, +y up, +z to the right, realAction_, realCurrentState_
  //pr2: +x forward, +y to the left, +z up, translatedAction, translatedCurrentState
  
  else if (realWorld == true){
    std::vector<double> translatedAction;
    translatedAction.push_back(realAction_[0]); //b to pr2: x+ to x+
    translatedAction.push_back(-realAction_[1]); // +z to +y
    //translatedAction.push_back(0.0);
    //translatedAction.push_back(0.0);
    translatedAction.push_back(0.0); //this is just +z for the pr2 which isn't read anyways
    
    sendRequest(translatedAction);
    //realCurrentState_.clear();
    //realCurrentState_.shrink_to_fit();
    
    std::vector<double> translatedCurrentState;
    std::cout << "Now, waiting for a response from the robot" << std::endl;
    while(!getResponse(translatedCurrentState)){
      //std::cout << "waiting for a response from the robot" << std::endl;
    }
    realCurrentState_[0] = translatedCurrentState[0]; //pr2 to b: x+ to x+
    realCurrentState_[1] = -translatedCurrentState[1]; // +y to +z
    std::cout << "observation from robot: " << realCurrentState_[0] << " , " << realCurrentState_[1] << std::endl;
  }
  
}


double basicBayes::randomDouble(){
  //srand((unsigned)time(NULL));
	double X=((double)rand()/(double)RAND_MAX);
	return X;
}

int main (int argc, char * const argv[]) {

	basicBayes filter;
	filter.filterSetup();
	filter.debug_print_ = false;

	/*
	std::vector<double> test1;
	std::vector<double> test2;

	double transArray[] = {0.1,0.0,0.0,0.1};
	std::vector<double> trans;
	trans.assign(transArray, transArray + sizeof(transArray)/sizeof(double));

	test1.push_back(3);
	test1.push_back(0);
	test2.push_back(-2);
	test2.push_back(0);
	std::cout << "blah" << filter.evaluteMVG(test1,test2,trans) << std::endl;
	*/
	/*
	MatrixXd m(2,2);
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	std::cout << m << std::endl;
	*/
	
	/*
	Eigen::Matrix<double,2,1> sampleVec;
	sampleVec << 0.0,0.1;
	Eigen::Matrix<double,2,1> meanVec;
	meanVec << 0.0,0.0;
	Eigen::Matrix<double,2,2> covMat;
	covMat << 1.0,0.0,0.0,1.0;
	*/
	
	/*
	Eigen::MatrixXd sampleVec(2,1);
	sampleVec << 0.0,0.1;	
	Eigen::MatrixXd meanVec(2,1);
	meanVec << 0.0,0.0;
	Eigen::MatrixXd covMat(2,2);
	covMat << 1.0,0.0,0.0,1.0;
	*/
	
	/*
	std::vector<double> sampleVecVect(3);
	sampleVecVect[0] = 0.0;
	sampleVecVect[1] = 0.0;
	sampleVecVect[2] = 0.0;
	
	std::vector<double> meanVecVect(3);
	meanVecVect[0] = 0.0;
	meanVecVect[1] = 0.0;
	meanVecVect[2] = 0.0;
	
	std::vector<double> covMatVect(9);
	covMatVect[0] = 1.0;
	covMatVect[1] = 0.0;
	covMatVect[2] = 0.0;
	covMatVect[3] = 0.0;
	covMatVect[4] = 1.0;
	covMatVect[5] = 0.0;
	covMatVect[6] = 0.0;
	covMatVect[7] = 0.0;
	covMatVect[8] = 1.0;
	*/

	/*
	Eigen::Map<Eigen::MatrixXd> sampleVec(&sampleVecVect[0],sampleVecVect.size(),1);
	Eigen::Map<Eigen::MatrixXd> meanVec(&meanVecVect[0],meanVecVect.size(),1);
	Eigen::Map<Eigen::MatrixXd> covMat(&covMatVect[0],meanVecVect.size(),meanVecVect.size());
	*/
	
	//double ans = filter.evaluteMVG(sampleVecVect,meanVecVect,filter.obsCovMat);
	//std::cout << ans << std::endl;
	
	
	filter.setupStates();
	filter.setupActions();
	std::vector<double> probCDF = filter.createCDF(filter.probList_);
	std::cout << "This is the CDF:" << std::endl;
	for (size_t i=0; i<probCDF.size(); i++){
	  std::cout << probCDF[i] << std::endl;
	}

	/*
	//moves
	std::vector< std::vector<double> > moveList;
	std::vector<double> move0;
	move0.push_back(0.15);
	move0.push_back(0.0);
	std::vector<double> move1 (2,0.15);
	std::vector<double> move2;
	move2.push_back(0.0);
	move2.push_back(0.15);
	std::vector<double> move3 (2,0.0);
	*/

       
	//moves 2
	std::vector< std::vector<double> > moveList;
	std::vector<double> move0;
	move0.push_back(-0.15);
	move0.push_back(-0.15);
	std::vector<double> move1;
	move1.push_back(-0.15);
	move1.push_back(0.15);
	std::vector<double> move2;
	move2.push_back(0.15);
	move2.push_back(0.15);
	std::vector<double> move3;
	move3.push_back(0.15);
	move3.push_back(-0.15);
	

	/*
	//big moves
	std::vector< std::vector<double> > moveList;
	std::vector<double> move0;
	move0.push_back(1.15);
	move0.push_back(0.0);
	std::vector<double> move1 (2,1.15);
	std::vector<double> move2;
	move2.push_back(0.0);
	move2.push_back(1.15);
	std::vector<double> move3 (2,0.0);
	*/

	moveList.push_back(move0);
	moveList.push_back(move1);
	moveList.push_back(move2);
	moveList.push_back(move3);

	std::vector<double> zeroState (2,0.0);
	//move[1] = 0.9;
	filter.realAction_ = zeroState;
	filter.realPrevState_ = zeroState;
	filter.realCurrentState_ = zeroState;
	//filter.runRealWorld();
	//std::vector<double> obs = filter.getObs();
	//std::vector<double> obs (2,0.8);
	//obs[0]=0.1;
	
	int filterIters = 4;
	std::vector<double> probM0;
	std::vector<double> probM1;
	std::vector<double> probM2;
	std::vector<double> probM3;

	/*
	double sum0 = std::accumulate(filter.probList.begin(),filter.probList.begin()+9,(double) 0);
	double sum1 = std::accumulate(filter.probList.begin()+9,filter.probList.begin()+10,(double) 0);
	double sum2 = std::accumulate(filter.probList.begin()+10,filter.probList.begin()+18,(double) 0);
	double sum3 = std::accumulate(filter.probList.end()-9,filter.probList.end(),(double) 0);
	*/

	std::vector<double> sumHold = filter.calcModelProb();

	double sum0 = sumHold[0];
	double sum1 = sumHold[1];
	double sum2 = sumHold[2];
	double sum3 = sumHold[3];
	
	std::cout << "M0: " << sum0 << std::endl;
	std::cout << "M1: " << sum1 << std::endl;
	std::cout << "M2: " << sum2 << std::endl;
	std::cout << "M3: " << sum3 << std::endl;
	std::cout << "Total: " << sum0+sum1+sum2+sum3 << std::endl;

	probM0.push_back(sum0);
	probM1.push_back(sum1);
	probM2.push_back(sum2);
	probM3.push_back(sum3);

	for (size_t i=0; i<filterIters; i++) {
	  
	  //old way
	  //std::vector<double> currentMove = moveList[i%4];
	  //filter.realAction_ = currentMove;
	  
	  //new way

	  //std::vector<double> save = filter.probList_;
	  filter.chooseAction();
	  //std::cout << "this be carazy: " << vectorDiff(save,filter.probList_) << std::endl;
	  //filter.printProbList();
	  std::cout << "Action chosen: " << filter.realAction_[0] << "," << filter.realAction_[1] << std::endl;



	  filter.realPrevState_ = filter.realCurrentState_;
	  //std::cout << "realPrevState_: " << filter.realPrevState_[0] << "," << filter.realPrevState_[1] << std::endl;
	  filter.runRealWorld();
	  //std::cout << "realCurrentState_: " << filter.realCurrentState_[0] << "," << filter.realCurrentState_[1] << std::endl;
	  filter.transitionUpdate(filter.realAction_);
	  //if you want noise added to the observation from the simulation
	  //filter.observationUpdate(filter.getObs());
	  //std::cout << "obs: " << filter.getObs()[0] << "," << filter.getObs()[1] << std::endl;
	  //if you are using the real robot
	  filter.observationUpdate(filter.realCurrentState_);

	  /*
	  double sumM0 = std::accumulate(filter.probList.begin(),filter.probList.begin()+9,(double) 0);
	  double sumM1 = std::accumulate(filter.probList.begin()+9,filter.probList.begin()+10,(double) 0);
	  double sumM2 = std::accumulate(filter.probList.begin()+10,filter.probList.begin()+18,(double) 0);
	  double sumM3 = std::accumulate(filter.probList.end()-9,filter.probList.end(),(double) 0);
	  */

	  std::vector<double> sumMHold = filter.calcModelProb();

	  double sumM0 = sumMHold[0];
	  double sumM1 = sumMHold[1];
	  double sumM2 = sumMHold[2];
	  double sumM3 = sumMHold[3];

	  std::cout << "M0: " << sumM0 << std::endl;
	  std::cout << "M1: " << sumM1 << std::endl;
	  std::cout << "M2: " << sumM2 << std::endl;
	  std::cout << "M3: " << sumM3 << std::endl;
	  std::cout << "Total: " << sumM0+sumM1+sumM2+sumM3 << std::endl;
	  probM0.push_back(sumM0);
	  probM1.push_back(sumM1);
	  probM2.push_back(sumM2);
	  probM3.push_back(sumM3);
	  
	}
	
	std::ofstream myfile ("example.txt");
	
	if (myfile.is_open()){
	  for (int i=0 ; i<probM0.size(); i++) {
	    myfile << probM0[i] << "\t" << probM1[i]<< "\t" << probM2[i] << "\t" << probM3[i] <<std::endl;
	  }
	  myfile.close();
	}
	
	else std::cout << "Unable to open output probability file" << std::endl;
	
	
	/*
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
	  
	  double X = randomDouble();
	  std::cout << X << std::endl;
	*/

	return 0;
}

