//this actually isn't a hash table test anymore. it's an ordered map.

#include <iostream>
#include <map>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "sasUtils.h"


typedef std::pair< std::vector<double>,std::vector<double> > pairVV;
typedef std::map< pairVV, std::vector<double> > mapPairVVV; //this is for stateActionState
typedef std::pair< pairVV, std::vector<double> > pairPairVVV;
typedef mapPairVVV::iterator mapPairVVV_it;

void insertIntoSAS(mapPairVVV& sas,std::vector<double> prevState, std::vector<double> action, std::vector<double> nextState){
  sas.insert(pairPairVVV (pairVV (prevState,action),nextState));
}

void readSASfromCSV(mapPairVVV& sas,std::string fileName){
  //temporary holder for the sas map
  mapPairVVV sasHolder;

  std::ifstream data(fileName.c_str());
  //these loops are under the specific assumption that the file is written with:
  //1) 8 numbers per line, separated by commas.
  //2) Each line is written in order of prevState, action, nextState
  //3) prevState is three numbers
  //4) action is two numbers
  //5) nextState is three numbers
  if (data.is_open()){

    std::string line;
    while(std::getline(data,line))
      {

	//initialize everyting you need for the inner loop
	std::stringstream  lineStream(line);
	std::string        cell;
	size_t count = 0;
	std::vector<double> tmpPrevState;
	std::vector<double> tmpAction;
	std::vector<double> tmpNextState;



	while(std::getline(lineStream,cell,','))
	  {
	    // You have a cell
	    std::stringstream cellStream(cell);
	    double result;
	    cellStream >> result;
	    if (count<3){
	      tmpPrevState.push_back(result);
	    }
	    else if (count>= 3 && count<=4){
	      tmpAction.push_back(result);
	    }
	    else if (count>4){
	      tmpNextState.push_back(result);
	    }
	    count++; //you have to keep a count of where you are
	  }
	
	insertIntoSAS(sasHolder,tmpPrevState,tmpAction,tmpNextState);

      } 
  }
  sas = sasHolder;
}

void writeSAStoCSV(mapPairVVV& sas,std::string fileName){
  //try to write to csv
  std::ofstream outFile(fileName.c_str());
  
  if (outFile.is_open()){
    for (mapPairVVV_it mapIt = sas.begin(); mapIt !=sas.end(); mapIt++){
      std::vector<double> tmpPrevState = mapIt->first.first;
      std::vector<double> tmpAction = mapIt->first.second;
      std::vector<double> tmpNextState = mapIt->second;
      for (size_t i=0;i<tmpPrevState.size();i++){
	outFile << tmpPrevState[i] << ",";
      }
      for (size_t i=0;i<tmpAction.size();i++){
	outFile << tmpAction[i] << ",";
      }
      for (size_t i=0;i<tmpNextState.size()-1;i++){
	outFile << tmpNextState[i] << ",";
      }
      outFile << tmpNextState.back() << "\n";
    }
    outFile.close();
  }
  else std::cout << "Unable to open output SAS file" << std::endl;
}

int main(){

  std::ifstream data2("files/SASout.csv");
  if (data2.good()){
    std::cout << "I didn't make it" << std::endl;
  }


  std::map<double,double> doublemap;
  doublemap.insert(std::pair<double,double>(5,10));
  doublemap.insert(std::pair<double,double>(4,8));
  doublemap.insert(std::pair<double,double>(6,12));
  doublemap[10] = 20;
  std::cout << std::distance(doublemap.find(7),doublemap.end()) << std::endl;
  std::cout << doublemap.count(7) << std::endl;
  std::cout << doublemap[7] << std::endl;
  doublemap[12] = 24;
  std::cout << doublemap.count(7) << std::endl;
  std::cout << std::distance(doublemap.find(7),doublemap.end()) << std::endl;
  //std::cout << doublemap.find(7)==doublemap.end() << std::endl;

  double d;
  std::cout << d << std::endl;
  std::cout << "next" << std::endl;

  std::map< std::pair< std::vector<double>,std::vector<double> >, std::vector<double> > stateActionState;

  std::vector< std::vector<double> > prevStates;
  std::vector< std::vector<double> > actions;
  std::vector< std::vector<double> > nextStates;

  std::vector<double> prevState1 (3,0.1);
  std::vector<double> prevState2 (3,0.2);
  std::vector<double> prevState3 (3,0.3);
  std::vector<double> prevState4 (3,0.4);

  std::vector<double> action1 (2,2);
  std::vector<double> action2 (2,0.5);  

  std::vector<double> nextState1 (3,0.2);
  std::vector<double> nextState2 (3,0.4);
  std::vector<double> nextState3 (3,0.6);
  std::vector<double> nextState4 (3,0.8);
  std::vector<double> nextState5 (3,0.05);
  std::vector<double> nextState6 (3,0.1);
  std::vector<double> nextState7 (3,0.15);
  std::vector<double> nextState8 (3,0.2);

  prevStates.push_back(prevState1);
  prevStates.push_back(prevState2);
  prevStates.push_back(prevState3);
  prevStates.push_back(prevState4);

  actions.push_back(action1);
  actions.push_back(action2);



  stateActionState.insert(std::pair< std::pair< std::vector<double>,std::vector<double> >, std::vector<double> > (std::pair< std::vector<double>,std::vector<double> > (prevState1,action1),nextState1));

  stateActionState.insert(std::pair< std::pair< std::vector<double>,std::vector<double> >, std::vector<double> > (std::pair< std::vector<double>,std::vector<double> > (prevState2,action1),nextState2));

  stateActionState.insert(std::pair< std::pair< std::vector<double>,std::vector<double> >, std::vector<double> > (std::pair< std::vector<double>,std::vector<double> > (prevState1,action2),nextState5));

  stateActionState.insert(std::pair< std::pair< std::vector<double>,std::vector<double> >, std::vector<double> > (std::pair< std::vector<double>,std::vector<double> > (prevState2,action2),nextState6));

  stateActionState.insert(pairPairVVV (pairVV (prevState3,action1),nextState3));

  std::vector<double> acceptVector = stateActionState[std::pair< std::vector<double>,std::vector<double> > (prevState2,action2)];

  for(size_t i=0; i<acceptVector.size(); i++){
    std::cout << acceptVector[i] << std::endl;
  }

  std::vector<double> acceptVector2 = stateActionState[pairVV (prevState3,action1)];

  for(size_t i=0; i<acceptVector2.size(); i++){
    std::cout << acceptVector2[i] << std::endl;
  }

  //try to do this stuff with a vector and map and all that jazzzzzzzz
  mapPairVVV sasHolder;

  std::ifstream data("files/SAS.csv");
  //these loops are under the specific assumption that the file is written with:
  //1) 8 numbers per line, separated by commas.
  //2) Each line is written in order of prevState, action, nextState
  //3) prevState is three numbers
  //4) action is two numbers
  //5) nextState is three numbers
  if (data.is_open()){

    std::string line;
    while(std::getline(data,line))
      {

	//initialize everyting you need for the inner loop
	std::stringstream  lineStream(line);
	std::string        cell;
	size_t count = 0;
	std::vector<double> tmpPrevState;
	std::vector<double> tmpAction;
	std::vector<double> tmpNextState;



	while(std::getline(lineStream,cell,','))
	  {
	    // You have a cell
	    std::stringstream cellStream(cell);
	    double result;
	    cellStream >> result;
	    if (count<3){
	      tmpPrevState.push_back(result);
	    }
	    else if (count>= 3 && count<=4){
	      tmpAction.push_back(result);
	    }
	    else if (count>4){
	      tmpNextState.push_back(result);
	    }
	    count++; //you have to keep a count of where you are
	  }
	
	insertIntoSAS(sasHolder,tmpPrevState,tmpAction,tmpNextState);

      } 
  }

  std::vector<double> acceptVector3 = sasHolder[pairVV (prevState2,action1)];
  std::cout << acceptVector3.size() << std::endl;
  for(size_t i=0; i<acceptVector3.size(); i++){
    std::cout << acceptVector3[i] << std::endl;
  }

  mapPairVVV sasAgain;
  readSASfromCSV(sasAgain,"files/SASout3.csv");

  std::vector<double> acceptVector4 = sasAgain[pairVV (prevState2,action1)];
  std::cout << "4: " << acceptVector4.size() << std::endl;
  for(size_t i=0; i<acceptVector4.size(); i++){
    std::cout << acceptVector4[i] << std::endl;
  }

  std::cout << "huh" << std::endl;


  //try to write to csv
  std::ofstream outFile("files/SASout.csv");

  std::cout << "huh" << std::endl;

  if (outFile.is_open()){
    for (mapPairVVV_it mapIt = sasAgain.begin(); mapIt !=sasAgain.end(); mapIt++){

      std::cout << "huh -1" << std::endl;

      std::vector<double> tmpPrevState = mapIt->first.first;
      std::vector<double> tmpAction = mapIt->first.second;
      std::vector<double> tmpNextState = mapIt->second;

      std::cout << "huh 0" << std::endl;

      for (size_t i=0;i<tmpPrevState.size();i++){
	outFile << tmpPrevState[i] << ",";
      }

      std::cout << "huh 1" << std::endl;

      for (size_t i=0;i<tmpAction.size();i++){
	outFile << tmpAction[i] << ",";
      }

      std::cout << "huh 2" << std::endl;
      std::cout << tmpNextState.size()-1 << std::endl;
      for (size_t i=0;i<tmpNextState.size()-1;i++){
	outFile << tmpNextState[i] << ",";
      }
      std::cout << "huh 3" << std::endl;

      outFile << tmpNextState.back() << "\n";
    }
    outFile.close();
  }
  else std::cout << "Unable to open output SAS file" << std::endl;

  std::cout << "huh" << std::endl;

  writeSAStoCSV(sasAgain,"files/SASout2.csv");
  mapPairVVV sas2;
  readSASfromCSV(sas2,"files/SASout2.csv");

  std::vector<double> acceptVector5 = sas2[pairVV (prevState2,action1)];
  std::cout << "5: " << acceptVector5.size() << std::endl;
  for(size_t i=0; i<acceptVector5.size(); i++){
    std::cout << acceptVector5[i] << std::endl;
  }

  std::cout << "Under this" << std::endl;

  sasUtils::writeSAStoCSV(sasAgain,"files/SASout3.csv");
  sasUtils::mapPairVVV sas3;
  sasUtils::readSASfromCSV(sas3,"files/SASout3.csv");

  /*
  std::vector<double> acceptVector6 = sasUtils::getFromSAS(sas3,prevState1,action1);
    std::cout << "6: " << acceptVector6.size() << std::endl;
  for(size_t i=0; i<acceptVector6.size(); i++){
    std::cout << acceptVector6[i] << std::endl;
  }
  */

  std::cout << "True or False" << std::endl;
  std::cout << sasUtils::isMatch(sas3,prevStates,actions) << std::endl;
  std::vector<double> blah (3,0.0);
  sasUtils::insertIntoSAS(sas3,prevState1,action1,nextState1);
  std::cout << sasUtils::isMatch(sas3,prevStates,actions) << std::endl;
  //sas3.erase(pairVV (blah,action1));
  //std::cout << sasUtils::isMatch(sas3,prevStates,actions) << std::endl;
  sas3.erase(pairVV (prevState1,action1));
  std::cout << sasUtils::isMatch(sas3,prevStates,actions) << std::endl;

  return 0;
}
