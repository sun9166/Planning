#ifndef __PATHPLAN_FAULT_VECTOR_HANDLER_H__
#define __PATHPLAN_FAULT_VECTOR_HANDLER_H__



#include <chrono>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <string>
#include <vector>
#include "common/base/macros.h"
#include "common/base/log/include/log.h"
#include "common_msgs.pb.h"


namespace acu {
namespace common {

#define MY_MAX(a, b)  (a)>(b)?(a):(b)

#define FAULT_VECTOR_SIZE 100


typedef common_msgs::FaultInfo Tfault;

class FaultVectorHandler
{
public:

	void PushFaultInfo(std::string node_name, int error_code,
	                   const std::string& msg, double timestamp,
	                   int level) {

		for (int i = 0; i < fault_vec_.size(); i++) {
			if (fault_vec_[i].error_code() == error_code)
				return;
		}
		Tfault fault;
		fault.set_module_name(node_name);
		fault.set_error_code(error_code);
		fault.set_msg(node_name + "/" + msg);
		fault.set_timestamp_sec(timestamp);
		fault.set_fault_level(level);
		//fault.module_name = node_name;
		//fault.error_code = error_code;
		//fault.msg = node_name + "/" + msg;
		//fault.timestamp_sec = timestamp;
		//fault.fault_level = level;
		if (fault_vec_.size() < FAULT_VECTOR_SIZE) fault_vec_.push_back(fault);
		max_module_fault_level = MY_MAX(level, max_module_fault_level);
		max_self_module_fault_level = MY_MAX(level, max_self_module_fault_level );
	}


	void PushFaultInfo(const Tfault &fault) {
		for (int i = 0; i < msg_fault_vec_.size(); i++) {
			if (msg_fault_vec_[i].error_code() == fault.error_code()) {
				msg_fault_vec_[i].set_timestamp_sec(fault.timestamp_sec());
				return;
			}
		}
		if (msg_fault_vec_.size() < FAULT_VECTOR_SIZE) msg_fault_vec_.push_back(fault);
	}

	void PushMsgFaultInfo() {
		for (auto &m : msg_fault_vec_) {
			if (fault_vec_.size() < FAULT_VECTOR_SIZE) fault_vec_.push_back(m);
			max_module_fault_level = MY_MAX(m.fault_level(), max_module_fault_level);//Use Module FaultLevel only.
		}
	}


	template <class Tpush>
	void PushFaultVecMsg(const Tpush & msg) {
		auto &ms = msg.header.fault_vec;
		auto it = msg_fault_vec_.begin();

		for (; it !=  msg_fault_vec_.end(); ) {
			bool find = false;
			for (int i = 0; i < ms.info_vec.size(); i++) {
				if (ms.info_vec[i].error_code == it->error_code) {
					find = true;
					break;
				}
			}
			if (!find) {
				it = msg_fault_vec_.erase(it);
			} else {
				++it;
			}
		}
		for (int i = 0; i < ms.info_vec.size(); i++) {
			PushFaultInfo(ms.info_vec[i]);
		}

		max_module_fault_level = MY_MAX(ms.module_fault_level, max_module_fault_level);
	}



	template <class Tpull>
	void PullFaultMsg(Tpull & msg) {
		msg.header.fault_vec.info_vec.clear();
		int reserve_size = msg_fault_vec_.size() + fault_vec_.size() + 1;
		if(msg_fault_vec_.size() > 0){
			fault_vec_.insert(fault_vec_.end(), msg_fault_vec_.begin(),
                               msg_fault_vec_.end());
		}
		msg.header.fault_vec.info_vec.reserve(reserve_size + 1);
		for (int i = 0; i < fault_vec_.size(); i++) {
			Tfault fault_info;
            fault_info.set_timestamp_sec(fault_vec_[i].timestamp_sec());
			fault_info.set_module_name(fault_vec_[i].module_name());
			fault_info.set_version(fault_vec_[i].version());
			fault_info.set_error_code(fault_vec_[i].error_code());
			fault_info.set_fault_level(fault_vec_[i].fault_level());
			fault_info.set_msg(fault_vec_[i].msg());
			fault_info.set_fault_type(fault_vec_[i].fault_type());

			//fault_info.timestamp_sec = fault_vec_[i].timestamp_sec;
			//fault_info.module_name = fault_vec_[i].module_name;
			//fault_info.version = fault_vec_[i].version;
			//fault_info.error_code = fault_vec_[i].error_code;
			//fault_info.fault_level = fault_vec_[i].fault_level;
			//fault_info.msg = fault_vec_[i].msg;
			//fault_info.fault_type = fault_vec_[i].fault_type;
			msg.header.fault_vec.info_vec.push_back(fault_info);
		}
		msg.header.fault_vec.module_fault_level = max_module_fault_level;

		fault_vec_.clear();
		max_self_module_fault_level = 0;
	}

	void ResetModuleFaultLevel() {
		max_module_fault_level = 0;
	}



public:
	int max_module_fault_level;
	int max_self_module_fault_level;
	std::vector<Tfault> fault_vec_;
	std::vector<Tfault> msg_fault_vec_;
	bool is_need_to_alarm;


private:
	FaultVectorHandler() {
		max_module_fault_level = 0;
		max_self_module_fault_level = 0;
		fault_vec_.clear();
		is_need_to_alarm = false;
		msg_fault_vec_.clear();

	}

	BASE_DECLARE_SINGLETON(FaultVectorHandler)

};

}
}













#endif