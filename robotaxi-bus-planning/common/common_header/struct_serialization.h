/*************************************************************************
    > File Name: serialization_struct.h
    > Author:xiayang
    > Mail:xiayang@idriverplus.com
    > Created Time: 2020年05月30日 星期五 17时32分54秒
 ************************************************************************/

#ifndef __SERIALIZATION_STRUCT_H__
#define __SERIALIZATION_STRUCT_H__
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <iomanip>

/***test**


typedef struct ServiceConfigItem
{
	int service_id;
	std::string service_name;
	std::string service_path;
	void Serialization(stringstream &ss) {
		REGISTER_SERIALIZATION_ATTRIBUTE( service_id, ',');
		REGISTER_SERIALIZATION_ATTRIBUTE(service_name, ',');
		REGISTER_SERIALIZATION_ATTRIBUTE(service_path, ',');
		REGISTER_ENDLINE();
	}

	void DeSerialization(stringstream &ss) {
		REGISTER_DESERIALIZATION_ATTRIBUTE( service_id, SPLIT_CHAR);
		REGISTER_DESERIALIZATION_ATTRIBUTE( service_name, SPLIT_CHAR);
		REGISTER_DESERIALIZATION_ATTRIBUTE( service_path, SPLIT_CHAR);
	}

	void print() {
		std::cout << "id|name|path:" << service_id << "|" << service_name << "|" << service_path << std::endl;
	}

} ServiceConfigItem;




*****/



using namespace std;


#define REGISTER_SERIALIZATION_ATTRIBUTE(name, split) \
        ss << setprecision(12)<<(name); \
        ss << split;

#define REGISTER_ENDLINE()\
        ss<<endl;

#define REGISTER_DESERIALIZATION_ATTRIBUTE(name, split) \
        {stringstream temp_ss;	\
        std::string temp_str;		\
        std::getline(ss, temp_str, split); 	\
        temp_ss << temp_str;				\
	    temp_ss >> name;} 							\





template<typename T>
bool SerializationStruct( T& data, string file_name) {
	ofstream of;
	of.open(file_name.c_str());
	if (!of) {
		cout << "open file error" << endl;
		return false;
	}
	stringstream ss;
	data.Serialization(ss);
	of << ss.str();
	of.close();
	return true;
}





template<typename T>
bool SerializationStructVector( vector<T>& data_vec, string file_name) {
	ofstream of;
	of.open(file_name.c_str());
	if (!of) {
		cout << "open file error" << endl;
		return false;
	}
	stringstream ss;
	for (int i = 0; i < data_vec.size(); i++) {
		data_vec[i].Serialization(ss);
	}
	of << ss.str();
	of.close();
	return true;
}





template<typename T>
bool DeSerializationStruct(string file_name,  T& data ) {
	ifstream in;
	in.open(file_name.c_str());
	if (!in) {
		cout << "open file error" << endl;
		return false;
	}
	string str;
	getline(in, str);
	std::cout << "str" << str << std::endl;
	stringstream ss ;
	ss << str;
	data.DeSerialization(ss);
	in.close();
	return true;
}


template<typename T>
bool DeSerializationStructFromStr(string content,  T& data ) {

	std::cout << "content:" << content << std::endl;
	stringstream ss ;
	ss << content;
	data.DeSerialization(ss);
	return true;
}


template<typename T>
bool DeSerializationStructVector(string file_name,  vector<T>& data_vec ) {
	data_vec.clear();
	ifstream in;
	in.open(file_name.c_str());
	if (!in) {
		cout << "open file error" << endl;
		return false;
	}
	while (!in.eof()) {
		string str;
		getline(in, str);
		if (str.length() == 0) break;

		stringstream ss ;
		ss << str;
		T data;
		data.DeSerialization(ss);
		data_vec.push_back(data);

	}
	in.close();
	return true;
}


template<typename T>
void Print(stringstream &ss, int space_size,  T data) {
	stringstream data_ss;
	data_ss  << data;
	int size = data_ss.str().length();
	if (space_size <= size) {
		ss << data;
		return;
	}
	for (int i = 0; i < (space_size - size) / 2; i++) {
		ss << " ";
	}
	ss << data;
	for (int i = 0; i < (space_size - size) / 2; i++) {
		ss << " ";
	}
}




#endif
