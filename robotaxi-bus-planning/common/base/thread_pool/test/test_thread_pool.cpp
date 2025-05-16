#include <thread>
#include <iostream>
#include <atomic>
#include <memory>
#include <unistd.h>
#include <vector>
#include "thread_pool.h"
using namespace std;

int TestWork(int times) {
	sleep(times);
	cout << "TestWork over" << endl;
	return 1000;
}

int TestWorkWhile() {
	while (1) {
		CHECK_CT_VALID(3);
		usleep(1000 * 100);
	}
	return 1000;
}


int main(int argc, char const *argv[])
{
	auto thread_pool = ThreadPool::Instance();
	eThreadStatus thread_state;
	//test init
	thread_pool->Init(10);
	cout << "init over" << endl;


	//test AddThreadWork
	for (int i = 0; i < 3; i++) {
		int thread_handle = thread_pool->getIdleThreadHandle();
		cout << "begin thread_handle :" << thread_handle << endl;
		const auto work = [&]() {
			return TestWork(10);
		};
		thread_pool->AddThreadWork(thread_handle, work);
	}


	//test StopWork
	int thread_handle = thread_pool->getIdleThreadHandle();
	cout << "StopWork thread_handle :" << thread_handle << endl;
	const auto work = [&]() {
		return TestWorkWhile();
	};
	thread_pool->AddThreadWork(thread_handle, work);
	sleep(2);
	while (thread_pool->StopWork(thread_handle) != 0) {
		cout << "stopping" << endl;
		sleep(1);
	}
	thread_state = thread_pool->CheckThreadState(thread_handle);
	if (thread_state != eThreadStatus::IDLE) {
		cout << "stopping error" << endl;
	} else {
		cout << "stopping success" << endl;
	}

	//test StopThread
	thread_handle = thread_pool->getIdleThreadHandle();
	cout << "StopThread thread_handle :" << thread_handle << endl;
	const auto stopping_work = [&]() {
		return TestWork(5);
	};
	thread_pool->AddThreadWork(thread_handle, stopping_work);
	while (thread_pool->StopThread(thread_handle) != 0) {
		cout << "stopping thread" << endl;
		sleep(1);
	}
	thread_state = thread_pool->CheckThreadState(thread_handle);
	if (thread_state != eThreadStatus::STOPPED) {
		cout << "stopping thread error" << endl;
	} else {
		cout << "stopping thread success" << endl;
	}


	// test CheckThreadState

	do {
		thread_state = thread_pool->CheckThreadState(0);
		cout << "CheckThreadState :" << (int)thread_state << endl;
		sleep(1);
	} while (thread_state != eThreadStatus::WAITING);

	//test GetWorkReturnValue
	int return_value = thread_pool->GetWorkReturnValue(0);
	cout << "GetWorkReturnValue :" << (int)return_value << endl;





	//test getIdleThreadHandle
	for (int i = 0; i < 100; i++) {
		int thread_handle = thread_pool->getIdleThreadHandle();
		cout << "thread_handle :" << thread_handle << endl;
	}


	sleep(100000);
	return 0;
}

