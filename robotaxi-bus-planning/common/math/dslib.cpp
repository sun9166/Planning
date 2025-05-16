/*************************************************************************
	> File Name: DSLib.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年10月26日 星期四 10时55分54秒
 ************************************************************************/

#include <iostream>
#include "dslib.h"
using namespace std;

namespace DS{
    DS_lib::DS_lib(){
        debug = false;
        statnum = -1;
        dims = 0;
        evidenceArray.clear();
        messAll.clear();
        finalBBA.clear();
        for(int i = 1; i < MAX; i++) 
        {
            a[i-1] = i;
            subsetFlag[i-1] = false;
        }
        stopFlag = false;
    }
	


    DS_lib::DS_lib(int statnum)
    {
        this->statnum = statnum;
        this->dims =  pow(2, statnum);
        this->debug = -1;
        messAll.resize(this->dims);
        evidenceArray.clear();
        finalBBA.resize(statnum);
        for(int i = 1; i < MAX; i++) 
        {          
            a[i-1] = i;
            subsetFlag[i-1] = false;
        }
        stopFlag = false;
		cout << "DS_lib init ok"<<endl;
    }

    void DS_lib::setStatNum(int statnum)
    {
        assert(statnum > 1);
        this->statnum = statnum;
        this->dims =  pow(2, statnum);
        this->debug = -1;
        messAll.resize(this->dims);
        evidenceArray.clear();
        finalBBA.resize(statnum);
        
    }


    void DS_lib::pushEvidence(vector<double> evidence)
    {
        assert(statnum > 1);
        assert(evidence.size() == dims);
        evidenceArray.push_back(evidence);
    }


    void DS_lib::process()
    {
        assert(evidenceArray.size() > 1);
        vector<double> vec = evidenceArray[0];
        for(int i = 1; i < evidenceArray.size(); i++)
        {
            vector<double> temp ;
            temp.resize(dims);
            for(int j = 0; j < dims; j++)
            {
                temp[j] = getMess(vec, evidenceArray[i], j);
            }
            vec = temp;
        }
         
    }

    void DS_lib::getAllCombination(int num)
    {

        assert(statnum > num);
        combinRes.clear();
        combine(a, statnum, num, b, num);
    }
    
    void DS_lib::combine(int a[], int n, int m, int b[], int M)
    {
        for(int i = n; i >= m; i--)
        {
            b[m-1] = i-1;
            if(m > 1)
                combine(a, i-1 , m-1, b, M);
            else{
                vector<int> indexVec;
                indexVec.clear();
                for(int j = M; j >= 0; j--)
                {
                    indexVec.push_back(a[b[j]]);
                }
                combinRes.push_back(indexVec);
            }
        }
    }


    void DS_lib::getComplemented()
    {
        assert(combinRes.size());
        for(int i = 0; i < combinRes.size(); i++)
        {
            vector<int> temp = getOneCompleted(combinRes[i]);
            completedRes.push_back(temp);
            
        }
    }

    vector<int> DS_lib::getOneCompleted(vector<int> now)
    {
        vector<int > res;
        assert(now.size() <= statnum);
        for(int i = 1; i <= statnum; i++)
        {
            bool matching = true;
            for(int j = 0; j < now.size();j++ )
            {
                if(i == now[j])
                {
                    matching = false;
                    break;
                }
            }
            if(matching)
            {
                res.push_back(i);
            }
        }
        return res;
    }

    int DS_lib::getLengthNum(int pos, int size, int preNum, int num)
    {
        int sum = 0;
        for(int i = preNum+1; i < num; i++)
        {
            int numRest = size - pos;
            int chooseNum = statnum - i;
            sum += getCombineNum(chooseNum,numRest );    
        }
        return sum;
    }

    int DS_lib::getIndexAccordCOntant(vector<int> &content)
	{
		if(content.size() == 0)
		    return 1;
		int size = content.size();
		int sum = 0;
		for(int i = 0; i < size; i++)
		{
		    int currenIndex = getCombineNum(statnum,i);
		    sum += currenIndex;
		}
		sort(content.begin(), content.end());
		int lastNum = 1;
		for(int i = 0; i < size; i++)
		{
			int preNum = 0;
			if(i > 0) preNum = 	content[i-1];
			lastNum += getLengthNum(i+1, size, preNum, content[i]);
			/*if(content[i-1]-i == 0){
				continue;
			}
			for(int j = i; j < content[i-1]; j++)
			{
				int numRest = size-i;
				int chooseNum = statnum - i;
				lastNum +=  getCombineNum(chooseNum,numRest );
			}*/
		
		}
		//lastNum += (content[size-1] - content[size-2]);
		sum += lastNum;
		return sum;

	}


    int DS_lib::getCombineNum(int a, int b)
    {
        assert(a>=b);
        int sum = 0;
        sum = fact(a)/(fact(b)*fact(a-b));
        return sum;
    }

    int DS_lib::fact(int num)
    {
        int sum = 1;
		if(num == 0) return 1;
        if(num == 1) return 1;
        sum = num * fact(num - 1);
        return sum;
    }

    void DS_lib::initialInput()
	{
		input.clear();
		for(int i = 1; i <=MAX; i++)
		{
			input.push_back(i-1);
		}
	}
    void DS_lib::QuerySubset(int i, int n, bool queryFlag)
    {
        if(stopFlag) return;
        if(i > n){
            vector<int> subVec;
           for(int j = 1; j <= n; j++)
            {
                if(subsetFlag[j])
                {
                    subVec.push_back(input[j]);
					//cout << input[j] << "\t";
                }
            }
			//cout << endl;
            if(queryFlag)
            {
				int query = getIndexAccordCOntant(subVec);
                if(query == needIndex)
                {
                    outQueryIndex = subVec;
                    stopFlag = true;
                }
            }
            outSubSet.push_back(subVec);
            return;    
        }
        subsetFlag[i] = true;
        QuerySubset(i+1, n, queryFlag);
        subsetFlag[i] = false;
        QuerySubset(i+1, n,queryFlag);

    }

    //index  from 1
    void DS_lib::QueryIndex(int index)
    {
        assert(index >= 1);
        assert(index <= dims);
        needIndex = index;
        stopFlag = false;
		initialInput();
		outSubSet.clear();
        QuerySubset(1,statnum,true);
    }

    

    //index from 0
    /*double DS_lib::getNullMess(int evidenceIndex1, int evidenceIndex2)
    {
        assert(evidenceIndex1 <= evidenceArray.size());
        assert(evidenceIndex2 <= evidenceArray.size());
        vector<double> evidenceVec1 = evidenceArray[evidenceIndex1];
        vector<double> evidenceVec2 = evidenceArray[evidenceIndex2];
        double sum = 0;
        for(int i = 1; i <= dims; i++)
        {
            double mess1 = 0.0, mess2 = 0.0;
            QueryIndex(i);
            if(outQueryIndex.size() == 0)
            {
                continue;
            }
            mess1 = evidenceVec1[i-1];
            input =  getOneCompleted(outQueryIndex);
            stopFlag = false;
            QuerySubset(1,input.size(),false);
            for(int j = 0; j < outSubSet.size(); j++)
            {
                int tempIndex = getIndexAccordCOntant(outSubSet[j]);
                mess2 += evidenceVec2[tempIndex -1];
            }
            sum += (mess1 * mess2);
        }
        return sum;
    }
*/

    void mergeVect(vector<int>& a, vector<int>& b)
    {
        for(int i = 0; i < b.size(); i++)
        {
            a.push_back(b[i]);
        }
    }

	void printVec(vector<int> a)
	{
		for(int i = 0; i < a.size(); i++)
		{
		    cout << a[i]<<"\t";
		}
		cout << endl;
	}
    //index frome 0
    double DS_lib::getMess(vector<double> evidenceVec1, vector<double> evidenceVec2, int index)
    {
        assert(evidenceVec1.size() == dims);
        assert(evidenceVec2.size() == dims);
       // vector<double> evidenceVec1 = evidenceArray[evidenceIndex1];
        //vector<double> evidenceVec2 = evidenceArray[evidenceIndex2];
        double sum = 0;
        
        QueryIndex(index+1);
        vector<int> commonVec = outQueryIndex;
        vector<int> rest = getOneCompleted(outQueryIndex);
        input = rest;
		input.insert(input.begin(),0);
        stopFlag = false;
		outSubSet.clear();
        QuerySubset(1, input.size()-1, false);
        vector<vector<int> > gRestVec = outSubSet;
        for(int i = 0; i < gRestVec.size(); i++)
        {
            double mess1 = 0.0, mess2 = 0.0;
            mergeVect(gRestVec[i], commonVec);
            int tempIndex = getIndexAccordCOntant(gRestVec[i]);
            mess1 = evidenceVec1[tempIndex-1];
			
            input = getOneCompleted(gRestVec[i]);
            stopFlag = false;
			outSubSet.clear();
			input.insert(input.begin(),0);
            QuerySubset(1, input.size()-1, false);
            vector<vector<int> > gRestVec1 = outSubSet;
            for(int j = 0; j < gRestVec1.size(); j++)
            {
                mergeVect(gRestVec1[j],commonVec);
                int tempIndex1 = getIndexAccordCOntant(gRestVec1[j]);
                mess2 += evidenceVec2[tempIndex1 - 1];
            }
            sum += (mess1 * mess2);
        }
        return sum;
    }



    vector<double> DS_lib::getMessArray(vector<double> vec1, vector<double> vec2)
    {
        assert(vec1.size() == dims);
        assert(vec2.size() == dims);
        vector<double> resVec;
        resVec.resize(dims);
        for(int i = 0; i< dims; i++)
        {
            double mess = getMess(vec1, vec2, i);
            resVec[i] = mess;
        }
        return resVec;
    }
}





