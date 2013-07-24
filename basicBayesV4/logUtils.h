#ifndef LOG_UTILS_H
#define LOG_UTILS_H

#include <vector>
#include "Eigen/Dense"


namespace logUtils {
  
  Eigen::Map<Eigen::MatrixXd> convertVect(std::vector<double>& vect);
  
  Eigen::Map<Eigen::MatrixXd> convertCovMat(std::vector<double>& covMat);

  double safe_log(double x);

  double safe_exp(double x);

  double logSumExp(std::vector<double> pLog);
  
  std::vector<double> normalizeVectorInLogSpace(std::vector<double> pLog);
  
  std::vector<double> expLogProbs(std::vector<double> logProbs);
  
  //This function is overloaded
  //Ver 1: pass in vectors
  double evaluteLogMVG(std::vector<double>& sampleVecVect, std::vector<double>& meanVecVect, std::vector<double>& covMatVect);
  
  //This function is overloaded
  //Ver 2: pass in matrices
  double evaluteLogMVG(Eigen::Map<Eigen::MatrixXd> sampleVec, Eigen::Map<Eigen::MatrixXd> meanVec, Eigen::Map<Eigen::MatrixXd> covMat);

}
  
#endif //LOG_UTILS_H
