#include <thread>
#include <iostream>
#include <atomic>
#include <memory>
#include <unistd.h>
#include <vector>
#include "thread_message_.h"
#include "socket_client.h"
#include "socket_server.h"

using namespace std;


typedef struct DataType
{
	int data[1024];
	int time;
	DataType() {
		time = 0;
	}
} DataType;




ThreadMsgList thread_msg_list(50);

void task_pub() {

	int i = 30;
	while (i--) {

		auto data_ptr = std::make_shared<ThreadMessage<DataType>>();
		data_ptr->content_.time = data_ptr->message_seq_;

		thread_msg_list.GetPushDataPtr(data_ptr);


		// // std::shared_ptr<ThreadMessage<DataType>> data_ptr = std::static_pointer_cast<ThreadMessage<DataType>>thread_msg_list.GetPushDataPtr();
		// if (data_ptr == nullptr) {
		// 	data_ptr = std::make_shared<ThreadMessage<DataType>>();
		// }
		// auto derive_ptr = std::static_pointer_cast<ThreadMessage<DataType>>(data_ptr);
		// if (derive_ptr == nullptr) {
		// 	cout << "nullptr" << endl;
		// }
		// derive_ptr->print();
		// derive_ptr->content_.time = i;

		cout << "seq:" << thread_msg_list.gdata_.size() << std::endl;
		for (int i = 0; i < thread_msg_list.gdata_.size(); i++) {
			cout << "\t" << thread_msg_list.gdata_[i]->message_seq_;
		}
		cout << endl;
		usleep(1000 * 500);
	}
}

void task_sub_1() {
	int i = 300;
	std::shared_ptr<MessageBase> data_ptr;
	int pull_index = -1;
	while (i--) {
		if (thread_msg_list.GetNextData(pull_index, data_ptr) == -1) {
			// cout << "cannot get data" << endl;
			usleep(1000 * 100);
			continue;
		}
		data_ptr->is_reading_.store(true);
		auto derive_ptr = std::static_pointer_cast<ThreadMessage<DataType>>(data_ptr);
		if (derive_ptr != nullptr) {
			cout << "task_sub_1 data " << derive_ptr->content_.time << endl;
		}
		usleep(1000 * 100);
		data_ptr->is_reading_.store(false);
	}
}

void task_sub_2() {
	int i = 300;
	std::shared_ptr<MessageBase> data_ptr;
	int pull_index = -1;
	while (i--) {
		while (thread_msg_list.GetNextData(pull_index, data_ptr) != -1) {
			data_ptr->is_reading_.store(true);
			auto derive_ptr = std::static_pointer_cast<ThreadMessage<DataType>>(data_ptr);
			if (derive_ptr != nullptr) {
				cout << "task_sub_2 data " << pull_index << "|" << derive_ptr->content_.time << endl;
			}
			data_ptr->is_reading_.store(false);
		}
		usleep(1000 * 1000);
	}
}

void task_sub_3() {
	int i = 300;
	std::shared_ptr<MessageBase> data_ptr;
	int pull_index = -1;
	while (i--) {
		while (thread_msg_list.GetNextData(pull_index, data_ptr) != -1) {
			data_ptr->is_reading_.store(true);
			auto derive_ptr = std::static_pointer_cast<ThreadMessage<DataType>>(data_ptr);
			if (derive_ptr != nullptr) {
				cout << "task_sub_3 data " << pull_index << "|" << derive_ptr->content_.time << endl;
			}
			data_ptr->is_reading_.store(false);
		}
		usleep(1000 * 3000);
	}
}

void task_sub_4() {

}


int main(int argc, char const *argv[])
{
	/* code */

	std::thread thread_pub, thread_sub_1, thread_sub_2, thread_sub_3, thread_sub_4 ;

	thread_pub = std::thread(task_pub);
	thread_sub_1 = std::thread(task_sub_1);
	thread_sub_2 = std::thread(task_sub_2);
	thread_sub_3 = std::thread(task_sub_3);

	thread_pub.join();
	thread_sub_1.join();
	thread_sub_2.join();
	thread_sub_3.join();
	std::cout << "hello" << std::endl;
	acu::common::SocketClient Sclient(1450);
	Sclient.InitSocket("192.168.1.1",8080);
	acu::common::SocketServer Sserver(1450);
	Sserver.InitServer(8081);
	return 0;
}



