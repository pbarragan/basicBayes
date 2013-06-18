//this one is in basicBayesV2
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
#include "basicBayes.h"
#include "BasicDemo.h"

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
	if (sum == -10.0) {
	  //Didn't do anything kind of this has to change
		nextState = prevState;
	}
	else {
		//this is saying there is no probability to transition between models
		if (prevState[0]==0.0) {
		  //WOM means without model.
		  std::vector<double> prevStateWOM;
		  
		  prevStateWOM.assign(prevState.begin()+1,prevState.end()); //state without the model which is always the first entry in the state
		  
		  BasicDemo world;
		  std::vector<double> tempWorldState = world.runSimulation(prevStateWOM,action);
		  nextState.push_back(0.0);
		  std::cout << "the simulation returned:" << std::endl;
		  for(size_t j=0;j<tempWorldState.size();j++){
		    std::cout << tempWorldState[j] << "," << std::endl;
		    nextState.push_back(tempWorldState[j]);
		  }
		}
		else if (prevState[0]==1.0) {
		  nextState=prevState;
		}
		
				
	}

	return nextState;
}


void basicBayes::transitionUpdate(std::vector<double> action){
	//in here, we need to for each state 1) do a state transition 2) drop a guassian on that as the mean, then for each state
	//3) calculate the probility of that state in that gaussian 4) multiple it by the prev probability of the state in the outer for loop
	//5) sum the probability added to each state as you go through the outer for loop
	//this will require a termporary probability list.
	
	std::cout << "Took an action: " << std::endl;
	for (size_t ll=0; ll<action.size(); ll++) {
		std::cout << action[ll] << ',';
	}
	std::cout << std::endl;
	
	std::vector<double> tempProbList (stateList.size(),0.0); //this will be the sum of the probability after each x_k-1 state

	for (size_t i=0; i<stateList.size(); i++) {
		//this loop is for x_k-1
		std::vector<double> tempStateProbList (stateList.size(),0.0); //this will hold the probability in the inner loop waiting for normalization

		std::vector<double> nextState = stateTransition(stateList[i], action); //this will be the mean of the guassian used to calculate the transition probability
		
		for (size_t j=0; j<stateList.size(); j++) {
			//this loop is for x_k
			tempStateProbList[j] = probState(stateList[j],nextState);
			std::cout << "probState: " << tempStateProbList[j] << std::endl;
		}
		tempStateProbList = normalizeVector(tempStateProbList); //normalize the distribution before you scale and add it.
		
		for (size_t k=0; k<stateList.size(); k++) {
			//this loop is for x_k
			tempProbList[k] += tempStateProbList[k]*probList[i];
		}
	}
	//this is debug stuff
	double sumBefore = std::accumulate(probList.begin(),probList.end(),(double) 0);
	probList = tempProbList;
	double sumAfter = std::accumulate(probList.begin(),probList.end(),(double) 0);

	std::cout << "Sum Before: " << sumBefore << std::endl;
	std::cout << "Sum After: " << sumAfter << std::endl;
	printProbList();

}


void basicBayes::observationUpdate(std::vector<double> obs){
	
	std::cout << "Got an observation" << std::endl;

	for (size_t i=0; i<stateList.size(); i++) {
		probList[i] *= probObs(obs,stateList[i]);
	}
	
	//normalize
	//printProbList();
	double sumBefore = std::accumulate(probList.begin(),probList.end(),(double) 0);

	probList = normalizeVector(probList);
	double sumAfter = std::accumulate(probList.begin(),probList.end(),(double) 0);
	std::cout << "Sum Before: " << sumBefore << std::endl;
	std::cout << "Sum After: " << sumAfter << std::endl;
	printProbList();
}
 
double basicBayes::probState(std::vector<double> sampleState, std::vector<double> meanState){
	//this funciton is problem specific
	
	//sampleState is the sample vector. meanState is the mean vector. This just drops a gaussian at the meanState with a constant covariance from the class.
	//return evaluteMVG(sampleState,meanState,transCovMat);
	
	if (meanState[0]==sampleState[0]){
		//WOM means without model.
		std::vector<double> sampleStateWOM;
		std::vector<double> meanStateWOM;
	
		sampleStateWOM.assign(sampleState.begin()+1,sampleState.end()); //state without the model which is always the first entry in the state 
		meanStateWOM.assign(meanState.begin()+1,meanState.end()); //state without the model which is always the first entry in the state
	
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
	
	stateWOM.assign(state.begin()+1,state.end()); //state without the model which is always the first entry in the state
		
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
	double obsArray[] = {0.1,0.0,0.0,0.1};
	double transArray[] = {0.1,0.0,0.0,0.1};
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
	double dimRanges[][2]={{0,1},{0.0,0.15},{0.0,0.15}};
	double dimNums[] = {2,2,2};
	int dims = sizeof(dimNums)/sizeof(dimNums[0]);
	
	std::vector< std::vector<double> > valueList2 = createValueList(dimRanges,dimNums,dims);
	stateList = recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueList2);
	
	//this section sets up the state probability as a uniform prior
	double tempProb = 1.0/stateList.size();
	probList.assign(stateList.size(),tempProb);
	
	//This section prints useful things
	int prod = 1;
	for (size_t jj = 0; jj<dims; jj++){
		prod *= dimNums[jj];
	}
	std::cout << "number of states expected: " << prod << std::endl;
	std::cout << "number of states: " << stateList.size() << std::endl;
	
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
	setCovMats(obsCovMat,transCovMat);

}

void basicBayes::printProbList(){
	std::cout << "Printing Probablity List:" << std::endl;
	for (size_t ii = 0; ii<probList.size(); ii++) {
		std::cout << probList[ii] <<std::endl;
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

std::vector<double> basicBayes::getObs(){
  std::vector<double> stateWNoise;
  for (size_t i=0; i<realCurrentState_.size(); i++){
    srand((unsigned)time(NULL));
    double X=((double)rand()/(double)RAND_MAX);
    stateWNoise.push_back(realCurrentState_[i]+0.1*X);
  }
  return stateWNoise;
}

void basicBayes::runRealWorld(){
  std::cout << "This is the real world, son" << std::endl;
  BasicDemo world;
  realCurrentState_ = world.runSimulation(realPrevState_,realAction_);
}


double randomDouble(){
  	srand((unsigned)time(NULL));
	double X=((double)rand()/(double)RAND_MAX);
	return X;
}

int main (int argc, char * const argv[]) {

	basicBayes filter;
	filter.filterSetup();
	
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
	
	/*
	Eigen::Map<Eigen::MatrixXd> sampleVec(&sampleVecVect[0],sampleVecVect.size(),1);
	Eigen::Map<Eigen::MatrixXd> meanVec(&meanVecVect[0],meanVecVect.size(),1);
	Eigen::Map<Eigen::MatrixXd> covMat(&covMatVect[0],meanVecVect.size(),meanVecVect.size());
	*/
	
	//double ans = filter.evaluteMVG(sampleVecVect,meanVecVect,filter.obsCovMat);
	//std::cout << ans << std::endl;
	
	filter.setupStates();
	
	std::vector<double> move (2,0.9);
	move[0]=0.15;
	move[1]=0.0;
	std::vector<double> zeroState (2,0.0);
	//move[1] = 0.9;
	filter.realAction_ = move;
	filter.realPrevState_ = zeroState;
	filter.realCurrentState_ = zeroState;
	//filter.runRealWorld();
	//std::vector<double> obs = filter.getObs();
	//std::vector<double> obs (2,0.8);
	//obs[0]=0.1;
	
	int filterIters = 1;
	std::vector<double> probM0;
	std::vector<double> probM1;
	
	double sum0 = std::accumulate(filter.probList.begin(),filter.probList.begin()+4,(double) 0);
	double sum1 = std::accumulate(filter.probList.end()-4,filter.probList.end(),(double) 0);
	
	std::cout << "M0: " << sum0 << std::endl;
	std::cout << "M1: " << sum1 << std::endl;
	probM0.push_back(sum0);
	probM1.push_back(sum1);
	
	for (size_t i=0; i<filterIters; i++) {
	  	filter.runRealWorld();

		filter.transitionUpdate(move);
		filter.observationUpdate(filter.getObs());
	
		double sumM0 = std::accumulate(filter.probList.begin(),filter.probList.begin()+4,(double) 0);
		double sumM1 = std::accumulate(filter.probList.end()-4,filter.probList.end(),(double) 0);
	
		std::cout << "M0: " << sumM0 << std::endl;
		std::cout << "M1: " << sumM1 << std::endl;
		probM0.push_back(sumM0);
		probM1.push_back(sumM1);
	}
	
	std::ofstream myfile ("example.txt");
	
	if (myfile.is_open()){
		for (int i=0 ; i<probM0.size(); i++) {
			myfile << probM0[i] << "\t" << probM1[i] << std::endl;
		}
		myfile.close();
	}
	
	else std::cout << "Unable to open file" << std::endl;
	

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

