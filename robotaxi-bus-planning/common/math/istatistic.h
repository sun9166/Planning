/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * author: wangxiao
 * update log:
 * 20170501:
 * 1) make up the basic static algorithm framework
*/
#ifndef CLASS_STATITIC
#define CLASS_STATITIC
#pragma once

//c++ lib
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#define MPI 3.1415926


using namespace std;

class istatistic 
{
public:
    explicit istatistic();
    virtual ~istatistic();
public:

    //description statistic
    double getMax(std::vector<double> input);
    double getMin(std::vector<double> input);
    double getSum(std::vector<double> input);
    double getMean(std::vector<double> input);
    double getVariance(std::vector<double> input);
    double getStdDev(std::vector<double> input);

};
#endif
