#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include "bbSide.h"

std::string vectorToTxtFile(std::vector<double> data){
  //this needs to get changed if you are using a different basicBayes package
  std::string fileName = "/mit/barragan/Documents/cppCode/basicBayes/basicBayesV4/files/bbToRobot.txt";
  std::ofstream bbToRobot;
  bbToRobot.open(fileName.c_str());
  if (bbToRobot.is_open()){
    for(size_t i=0;i<data.size();i++){
      bbToRobot << data[i] << "\n";
    }
    bbToRobot.close();
  }
  else {
    //std::cout << "vectorToTxtFile: Unable to open output file" << std::endl;
  }
  return fileName;
}

std::vector<double> txtFileToVector(std::string fileName) {
	//This function reads in a .txt file of plain data
	//with 1 value on each line and converts that value
	//and places it in a vector.
	
	//This is the setup to read the file containing the data
  std::string line;
  std::ifstream myfile(fileName.c_str());
  double x;
  std::vector<double> data;
  
  //This loop will read the file line by line, convert each number, and build a vector
  if (myfile.is_open())
    {
      while(myfile.good())
	{
	  //if( myfile.eof() ) break;
	  //std::cout << myfile.eof() << std::endl;
	  //std::string line;
	  //double x;
	  getline(myfile,line);
	  if (line != ""){
	    std::istringstream i(line);
	    i >> x;
	    data.push_back(x);
	    //std::cout << line << std::endl;
	    //std::cout << x << std::endl;
	  }
	}
      myfile.close();
      std::cout << "txtFileToVector: Input file opened successfully" << std::endl;
    }
  
  else{
    //std::cout << "txtFileToVector: Unable to open input file" << std::endl;
  }
  return data;
}

void sendRequest(std::vector<double> request){
  std::string sentFile = vectorToTxtFile(request);
}

bool getResponse(std::vector<double> &dataVec){
  //this needs to get changed if you are using a different basicBayes package
  std::string receivedFile = "/mit/barragan/Documents/cppCode/basicBayes/basicBayesV4/files/robotToBB.txt";
  std::vector<double> data = txtFileToVector(receivedFile);
  if(data.size()!=0){
    for(size_t i = 0; i<data.size();i++){
      //std::cout << "what: " << data[i] << std::endl;
      //std::cout << data.size() << std::endl;
    }
    dataVec = data;
    remove(receivedFile.c_str());
    return true;
  }
  return false;
}

/*
int main(){
  
  //create action
  std::vector<double> action;
  action.push_back(0.70);
  action.push_back(0.10);
  action.push_back(0.00);
  
  sendRequest(action);
  std::cout << "main func" << std::endl;
  while(!getResponse()){}
  return 0;
}
*/
