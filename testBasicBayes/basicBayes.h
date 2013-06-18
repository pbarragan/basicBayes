/*
 *  basicBayes.h
 *  basicBayes
 *
 *  Created by Patrick Barragan on 12/17/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include <vector>

#ifndef BASIC_BAYES_H
#define BASIC_BAYES_H

class basicBayes {

	public:
	
	std::vector<double> normalizeVector(std::vector<double> probVect);
	
	std::vector<double> stateTransition(std::vector<double> prevState, std::vector<double> action);

	void transitionUpdate(std::vector<double> action);

	void observationUpdate(std::vector<double> obs);
	
	double probState(std::vector<double> sampleState, std::vector<double> meanState);
	
	double probObs(std::vector<double> obs, std::vector<double> state);

	Eigen::Map<Eigen::MatrixXd> convertVect(std::vector<double>& vect);

	Eigen::Map<Eigen::MatrixXd> convertCovMat(std::vector<double>& covMat);

	double evaluteMVG(std::vector<double>& sampleVecVect, std::vector<double>& meanVecVect, std::vector<double>& covMatVect);
	
	void setCovMats(std::vector<double>& obs,std::vector<double>& trans);
	
	void setupStates();
	
	void filterSetup();
	
	std::vector< std::vector<double> > createValueList(double dimRanges[][2], double dimNums[], int dims);

	std::vector< std::vector<double> > recurseList(std::vector< std::vector<double> > totalList, std::vector<double> oldSeq, int level, std::vector< std::vector<double> > valueList);

	
	std::vector<double> obsCovMat;
	std::vector<double> transCovMat;
	std::vector< std::vector<double> > stateList;
	std::vector<double> probList;
	
	void printProbList();
	void printStateList();

};

#endif //BASIC_BAYES_H
