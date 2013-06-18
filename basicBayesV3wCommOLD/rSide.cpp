#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

std::string vectorToTxtFile(std::vector<double> data){
  std::string fileName = "/Users/barragan/Documents/cppCode/LIS/fileInterfaceTest/files/robotToBB.txt";
  std::ofstream robotToBB;
  robotToBB.open(fileName.c_str());
  if (robotToBB.is_open()){
    for(size_t i=0;i<data.size();i++){
      robotToBB << data[i] << "\n";
    }
    robotToBB.close();
  }
  else std::cout << "Unable to open output file" << std::endl;
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
	    std::cout << line << std::endl;
	    std::cout << x << std::endl;
	  }
	}
      myfile.close();
      std::cout << "Input file opened successfully" << std::endl;
    }
  
  //else std::cout << "Unable to open input file" << std::endl;
  
  return data;
}

void sendResponse(std::vector<double> response){
  std::string sentFile = vectorToTxtFile(response);
}

bool getRequest(){
  std::string receivedFile = "/Users/barragan/Documents/cppCode/LIS/fileInterfaceTest/files/bbToRobot.txt";
  std::vector<double> data = txtFileToVector(receivedFile);
  if(data.size()!=0){
    std::vector<double> newData;
    for(size_t i = 0; i<data.size();i++){
      std::cout << "what: " << data[i]*2.0 << std::endl;
      std::cout << data.size() << std::endl;
      newData.push_back(2.0*data[i]);
    }
    remove(receivedFile.c_str());
    
    sendResponse(newData);
    return true;
  }
  return false;
}

int main(){
  
  /*
  //create action
  std::vector<double> action;
  action.push_back(1.1);
  action.push_back(0.9);
  
  //std::string fileName = vectorToTxtFile(action);
  //std::vector<double> data = txtFileToVector(fileName);
  
  for(size_t i = 0; i<data.size();i++){
    std::cout << "what: " << data[i]*2.0 << std::endl;
    std::cout << data.size() << std::endl;
  }
  */

  while(true){
    getRequest();
  }

  return 0;
}
