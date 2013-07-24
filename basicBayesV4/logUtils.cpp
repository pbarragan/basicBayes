#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include <vector>
#include <algorithm>
#include <math.h>
#include <limits>
#include "logUtils.h"

using Eigen::MatrixXd;


Eigen::Map<Eigen::MatrixXd> logUtils::convertVect(std::vector<double>& vect)
{
	Eigen::Map<Eigen::MatrixXd> returnVec(&vect[0],vect.size(),1);
	return returnVec;
}

Eigen::Map<Eigen::MatrixXd> logUtils::convertCovMat(std::vector<double>& covMat)
{
	Eigen::Map<Eigen::MatrixXd> returnCovMat(&covMat[0],sqrt(covMat.size()),sqrt(covMat.size()));
	return returnCovMat;
}

//Returns nan if x is negative, -inf (-DBL_MAX from float.h) if x is 0, and the regular log(x) otherwise.
double logUtils::safe_log(double x){
  if (x < 0.0){
    std::cout << "Hey Silly, you're trying to take the log of a negative number. Silly." << std::endl;
    return NAN;
  }
  else if (x == 0){
    return -std::numeric_limits<double>::infinity();
  }
  else {
    return log(x);
  }
}

//Returns 0 if x is -inf (-DBL_MAX from float.h) and the regular exp(x) otherwise.
double logUtils::safe_exp(double x){
  if (x == -std::numeric_limits<double>::infinity()){
    return 0.0;
  }
  else {
    return exp(x);
  }
}

//Use the log-sum-exp trick to compute log sum_i exp(v_i), for v_i in v
double logUtils::logSumExp(std::vector<double> pLog){
  double maxVal = *std::max_element(pLog.begin(),pLog.end());
  double sum = 0;
  for(size_t i = 0; i<pLog.size(); i++){
    sum += safe_exp(pLog[i]-maxVal);
  }
  return maxVal + log(sum);
}

//Given an unnormalized distribution in log probability format,
//uses log-sum-exp to normalize the distribution in log-space
std::vector<double> logUtils::normalizeVectorInLogSpace(std::vector<double> pLog){
  double logNormalizer = logSumExp(pLog);
  std::vector<double> pLogNormalized;
  for(size_t i=0; i<pLog.size(); i++){
    pLogNormalized.push_back(pLog[i]-logNormalizer);
  }
  return pLogNormalized;
}

//Given a set of log probabilities, return a list of probabilities after exponentiation
std::vector<double> logUtils::expLogProbs(std::vector<double> logProbs){
  std::vector<double> probs;
  for(size_t i=0; i<logProbs.size(); i++){
    probs.push_back(safe_exp(logProbs[i]));
  }
  return probs;
}

//This function is overloaded
//Ver 1: pass in vectors
double logUtils::evaluteLogMVG(std::vector<double>& sampleVecVect, std::vector<double>& meanVecVect, std::vector<double>& covMatVect){
	//this funciton is problem specific
	//Convert
	Eigen::Map<Eigen::MatrixXd> sampleVec = convertVect(sampleVecVect);
	Eigen::Map<Eigen::MatrixXd> meanVec = convertVect(meanVecVect);
	Eigen::Map<Eigen::MatrixXd> covMat = convertCovMat(covMatVect);
	
	Eigen::MatrixXd error = (sampleVec - meanVec);
	Eigen::Matrix<double,1,1> secondHalf = (error.transpose()*covMat.inverse()*error);
	double secondHalfDBL = secondHalf(0); //this is a hack
//return (1.0/(pow(2.0*M_PI,meanVec.rows()/2.0)*pow(covMat.determinant(),0.5)))*exp(-0.5*secondHalf(0,0));
	return -0.5*(meanVec.rows()*safe_log(2*M_PI)+safe_log(covMat.determinant())+secondHalfDBL);
}

//This function is overloaded
//Ver 2: pass in matrices
double logUtils::evaluteLogMVG(Eigen::Map<Eigen::MatrixXd> sampleVec, Eigen::Map<Eigen::MatrixXd> meanVec, Eigen::Map<Eigen::MatrixXd> covMat){
	//this funciton is problem specific
	
	Eigen::MatrixXd error = (sampleVec - meanVec);
	Eigen::Matrix<double,1,1> secondHalf = (error.transpose()*covMat.inverse()*error);
	double secondHalfDBL = secondHalf(0); //this is a hack
	//return (1.0/(pow(2.0*M_PI,meanVec.rows()/2.0)*pow(covMat.determinant(),0.5)))*exp(-0.5*secondHalf(0,0));
	return -0.5*(meanVec.rows()*safe_log(2*M_PI)+safe_log(covMat.determinant())+secondHalfDBL);
}


