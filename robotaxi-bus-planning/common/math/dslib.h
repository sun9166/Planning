/*************************************************************************
	> File Name: DSLib.h
	> Author: 
	> Mail: 
	> Created Time: 2017年10月26日 星期四 10时56分00秒
 ************************************************************************/

#ifndef _DSLIB_H
#define _DSLIB_H
#include <iostream>
#include <vector>
#include <math.h>
#include <assert.h>
#include <algorithm>
using namespace std;
#define MAX 100

namespace DS{
    class DS_lib{
        public:
            DS_lib();
            DS_lib(int statnum);
            void setStatNum(int num);
            void pushEvidence(vector<double> evidence);
            void process(); 
            vector<double> getMessAll();
            vector<double> getBBA();
        //private:
            void getAllCombination(int num);
            void getComplemented(); 
            void combine(int a[], int n, int m, int b[], int M);
            vector<int> getOneCompleted(vector<int> now);
            int getIndexAccordCOntant(vector<int> &);
            int getLengthNum(int pos, int size, int preNum, int nowNum);
            int getCombineNum(int a, int b);//for c b
                                            //     a
            int fact(int num);
            void subSet();
            void QuerySubset(int i, int n, bool isQuery);

            void QueryIndex(int index);




            vector<double> getMessArray(vector<double> vec1, vector<double> vec2);
            double getMess(vector<double> evidenceVec1, vector<double> evidenceVec2, int index);
        //private:
            int statnum;
            int dims;
            int debug;
            vector< vector<double> > evidenceArray;
            vector<double> messAll;
            vector<double> finalBBA;

            int a[MAX], b[MAX];
            vector<vector < int> > combinRes; 
            vector<vector <int> >  completedRes;
            bool subsetFlag[MAX];

            //subSet
            bool stopFlag;
            vector<int> input;
			void initialInput();
            vector< vector<int> > outSubSet;

            //QueryIndex result
            int needIndex;
            vector<int> outQueryIndex;
    };
}




#endif
