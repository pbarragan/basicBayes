#ifndef SAS_UTILS_H
#define SAS_UTILS_H

#include <vector>
#include <string>
#include <map>

namespace sasUtils {

  typedef std::pair< std::vector<double>,std::vector<double> > pairVV;
  typedef std::map< pairVV, std::vector<double> > mapPairVVV; //this is for stateActionState
  typedef std::pair< pairVV, std::vector<double> > pairPairVVV;
  typedef mapPairVVV::iterator mapPairVVV_it;
  
  void insertIntoSAS(mapPairVVV& sas,std::vector<double> prevState, std::vector<double> action, std::vector<double> nextState);

  std::vector<double> getFromSAS(mapPairVVV& sas,std::vector<double> prevState, std::vector<double> action);

  bool isMatch(mapPairVVV& sas, std::vector< std::vector<double> > states, std::vector< std::vector<double> > actions);

  bool readSASfromCSV(mapPairVVV& sas,std::string fileName);
  
  bool writeSAStoCSV(mapPairVVV& sas,std::string fileName);

}


#endif //SAS_UTILS_H
