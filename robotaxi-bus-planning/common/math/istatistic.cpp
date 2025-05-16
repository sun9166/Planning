#include "istatistic.h"

istatistic::istatistic(){
  
}
istatistic::~istatistic(){
	
}
//
double istatistic::getMax(std::vector<double> input){
     std::sort(input.begin(),input.end());
     return input[input.size()-1];
}
//
double istatistic::getMin(std::vector<double> input){
     std::sort(input.begin(),input.end());
     return input[0];
}
//
double istatistic::getSum(std::vector<double> input){
     double sum = 0.0;
     for(int i=0;i<input.size();i++){
         sum += input[i];
     }
     return sum;
}
//
double istatistic::getMean(std::vector<double> input){
     double sum = getSum(input);
     return sum/input.size();
}
//
double istatistic::getVariance(std::vector<double> input){
     double mean = getMean(input);
     double temp = 0.0;
     for(int i=0;i<input.size();i++){
         temp+=(input[i]-mean)*(input[i]-mean);
    }
    return temp/input.size();
}
//
double istatistic::getStdDev(std::vector<double> input){
     return sqrt(getVariance(input));
}
