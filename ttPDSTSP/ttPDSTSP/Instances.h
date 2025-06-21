#pragma once
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
class INSTANCE
{
public:
    vector<vector<double>> tau;
    vector<vector<double>> tauprime;
    vector<vector<double>> nodes;
    vector<int> Cprime;
    vector<int> truckonly;
    vector<int> C;
    vector<double> dDrones;
    int UAVs;
    int n;
    bool loadFromFile(const std::string& filename = "att48_0_2_1_1.txt");
    void displayData();
};
double tinhkc(const vector<vector<double>>& mat, int node);