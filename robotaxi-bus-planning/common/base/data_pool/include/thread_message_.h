#ifndef __THREAD_MESSAGE_H__
#define __THREAD_MESSAGE_H__


#include <atomic>
#include <unistd.h>
#include <list>
#include "base/macros.h"
#include <memory>
#include <vector>
#include <iostream>
#include <map>
using namespace std;

class MessageBase
{
public:
	MessageBase() {
		static int sequeue_num = 0;
		message_seq_ = sequeue_num++;
		is_reading_.store(false);
	}

	MessageBase(const MessageBase &t) {
		is_reading_.store(false);
		message_seq_ = t.message_seq_;
	}
	~MessageBase() {

	}

	void operator=(const MessageBase &t) {
		this->message_seq_ = t.message_seq_;
		this->is_reading_.store(false);
	}

public:
	int message_seq_;
	std::atomic<bool>  is_reading_;
};



template <class T>
class ThreadMessage: public MessageBase
{
public:
	ThreadMessage() {
	}
	ThreadMessage(const ThreadMessage &t) {
		is_reading_.store(false);
		message_seq_ = t.message_seq_;
	}
	~ThreadMessage() {}

	void operator=(const ThreadMessage &t) {
		this->message_seq_ = t.message_seq_;
		this->is_reading_.store(false);
		this->content_ = t.content_;
	}
	void print() {

		content_.time = 99;
		std::cout << "hello" << std::endl;
	}

public:
	T content_;
};




class ThreadMsgList
{
public:
	ThreadMsgList(int length = 10) {
		max_size_ = 10;
		push_index_ = -1;
		// pull_index_ = -1;
		if (length > 0) {
			max_size_ = length;
		}
		gdata_.reserve(max_size_ + 2);
	}

	~ThreadMsgList() {}

	int GetPushDataPtr(const std::shared_ptr<MessageBase> msg_ptr) {
		// data.seq_
		while (1) {
			push_index_ = (push_index_ + 1) % max_size_;
			if (push_index_ == gdata_.size()) {

				gdata_.push_back( msg_ptr);
				std::cout << "GetPushDataPtr index " << push_index_  << std::endl;
				return 0;
			}
			if (gdata_.at(push_index_)->is_reading_.load() == false) {
				std::cout << "GetPushDataPtr index " << push_index_ << std::endl;
				gdata_[push_index_] = msg_ptr;
				return 0;
			}
		}
		return -1;
	}

	int GetNextData(int &pull_index, std::shared_ptr<MessageBase> &data) {
		if (pull_index < 0) {
			if (gdata_.size() == 0) {
				return -1;
			}
			data = gdata_.at(0);
			pull_index = 0;
			return 0;
		}
		int try_times = max_size_ + 1;

		int try_index = pull_index;
		while (try_times--) {

			int next = (try_index + 1) % max_size_;
			// std::cout << "pull_index " << pull_index << "|" << next << "|" << gdata_.size() << std::endl;
			if (next >= (int)gdata_.size()) return -1;
			// std::cout << "seq " << gdata_.at(next)->message_seq_ << "|" << gdata_.at(pull_index)->message_seq_ << std::endl;
			if (gdata_.at(next)->message_seq_ > gdata_.at(pull_index)->message_seq_) {
				pull_index = next;
				data = gdata_.at(pull_index);
				return 0;
			}
			try_index = next;

		}
		return -1;
	}


public:
	std::vector<std::shared_ptr<MessageBase>> gdata_;

private:
	int push_index_;
	int max_size_;
};

class ThreadMessageSub
{
public:
	ThreadMessageSub() {

	}
	~ThreadMessageSub() {

	}


	int pull_index;

};



class ChannelMessage
{
public:
	int AddNewChannel(const std::string& channel_name, int length) {
		auto  iter = channel_message_map_.find(channel_name);
		if (iter != channel_message_map_.end()) {
			return -1;
		}
		std::shared_ptr<ThreadMsgList> ptr = std::make_shared<ThreadMsgList>(length);
		channel_message_map_.insert(std::pair<string, std::shared_ptr<ThreadMsgList>>(channel_name, ptr));
		return 0;
	}

	int PushChannelData(const std::string& channel_name, const std::shared_ptr<MessageBase> msg_ptr) {
		auto  iter = channel_message_map_.find(channel_name);
		if (iter == channel_message_map_.end()) {
			return -1;
		}
		iter->second->GetPushDataPtr(msg_ptr);
		return 0;
	}

	int PullChannelData(int &index, const std::string& channel_name, std::shared_ptr<MessageBase> &msg_ptr) {
		auto  iter = channel_message_map_.find(channel_name);
		if (iter == channel_message_map_.end()) {
			return -1;
		}
		return iter->second->GetNextData(index, msg_ptr);
	}
private:
	std::map<string,  std::shared_ptr<ThreadMsgList> > channel_message_map_;

private:
	ChannelMessage() {

	}



	BASE_DECLARE_SINGLETON(ChannelMessage);

};




#endif