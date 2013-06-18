#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

std::string vectorToTxtFile(std::vector<double> data);

std::vector<double> txtFileToVector(std::string fileName);

void sendRequest(std::vector<double> request);

bool getResponse(std::vector<double> &dataVec);
