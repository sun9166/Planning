#ifndef __CPU_SELECT_HPP__
#define __CPU_SELECT_HPP__
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/sysinfo.h>

#include <iostream>
#include "common/common_header/global_define.h"

namespace acu {
namespace common {


class CpuSelect
{
public:
	CpuSelect() {};
	~CpuSelect() {};


	static void BindProgressCpuCore(int cpu_id)
	{
		if (cpu_id < 0) return;
		if (cpu_id >= get_nprocs() || cpu_id < 0) {
			printf("Warning: cpu id %d is invalid\n", cpu_id);
			return;
		}
		cpu_set_t mask;
		//! sched_setaffinity
		CPU_ZERO(&mask);
		CPU_SET(cpu_id, &mask);
		if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) < 0) {
			printf("Error: cpu id %d sched_setaffinity\n", cpu_id);
		}
	}


	static int GetProgressCpuCore()
	{
		cpu_set_t get;
		CPU_ZERO(&get);
		if (sched_getaffinity(0, sizeof(get), &get) == -1)//获取线程CPU亲和力
		{
			printf("Warning: cound not get thread affinity, continuing...\n");
			return -1;
		}
		for (int i = 0; i < get_nprocs(); i++)
		{
			if (CPU_ISSET(i, &get))//判断线程与哪个CPU有亲和力
			{
				return i;
			}
		}
		return -1;
	}


	static void BindThreadCpuCore(int cpu_id)
	{
		if (cpu_id < 0) return;
		if (cpu_id >= get_nprocs() || cpu_id < 0) {
			printf("Warning: cpu id %d is invalid\n", cpu_id);
			return;
		}
		cpu_set_t mask;
		//! sched_setaffinity
		CPU_ZERO(&mask);
		CPU_SET(cpu_id, &mask);

		if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0) {
			printf("Error:BindThreadCpuCore cpu id %d sched_setaffinity\n", cpu_id);
		}
		return;
	}


	static int GetThreadCpuCore()
	{
		cpu_set_t get;
		CPU_ZERO(&get);
		if (pthread_getaffinity_np(pthread_self(), sizeof(get), &get) < 0) {
			fprintf(stderr, "get thread affinity failed\n");
		}

		/* 查看当前线程所运行的所有cpu*/
		for (int i = 0; i < get_nprocs(); i++) {
			if (CPU_ISSET(i, &get)) {
				return i;
			}
		}

		return -1;
	}

	//SCHED_FIFO  SCHED_RR SCHED_OTHER
	static void setProgressAttribute(int core_num,  int priority, int sched_policy = SCHED_RR) {
		BindProgressCpuCore(core_num);
		pid_t pid = getpid();
		struct sched_param param;
		param.sched_priority = priority;
		extern int errno;
		int ret = sched_setscheduler(pid, sched_policy, &param);
		if ( ret != 0) {
			std::cout << " setProgressAttribute sched_setscheduler failed " << ret << "|" << errno << std::endl;
		}
	}

	static void setPidProgressAttribute(int pid,   int priority, int sched_policy = SCHED_RR) {
		struct sched_param param;
		param.sched_priority = priority;
		extern int errno;
		int ret = sched_setscheduler(pid, sched_policy, &param);
		if ( ret != 0) {
			std::cout << " setPidProgressAttribute sched_setscheduler failed " << ret << "|" << errno << std::endl;
		}
	}


	static void setThreadAttribute(int core_num, int sched_policy, int priority) {
		BindThreadCpuCore(core_num);
		struct sched_param param;
		param.sched_priority = priority;
		pthread_setschedparam(pthread_self(), sched_policy, &param);
	}


	static void setThreadPriority(pthread_t *thread, int priority) {
		struct sched_param my_param;
		int my_policy;
		int status;
		pthread_getschedparam(*thread, &my_policy, &my_param);
		my_param.sched_priority = priority;
		if (pthread_setschedparam(*thread, my_policy, &my_param) != 0) //线程运行时设置策略和优先值
			printf("pthread_setschedparam failed\n");
	}




};





}
}



#endif

