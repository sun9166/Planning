#ifndef __ACU_COMMON_SEMPORE_H__
#define __ACU_COMMON_SEMPORE_H__

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/sem.h>
#include <iostream>
#include <string>
#include <errno.h>



namespace acu {
namespace common {


union USemun
{
	int val;
	struct semid_ds *buf;
	unsigned short *arry;
};

class SemporeTransmitter
{
public:
	SemporeTransmitter() {

	}
	~SemporeTransmitter(){
		
	}
private:
	static int BKDRHash(const char *str)
	{
		unsigned int seed = 131; // 31 131 1313 13131 131313 etc..
		unsigned int hash = 0;

		while (*str)
		{
			hash = hash * seed + (*str++);
		}

		return (hash & 0x7FFFFFFF);
	}
public:

	static int GetSemId(std::string sem_topic) {
		int bkd_value = BKDRHash(sem_topic.c_str());
		int sem_id = semget((key_t)bkd_value, 1, 0666 | IPC_CREAT);
		return  sem_id;
	}
	static int InitSemvalue(std::string sem_topic, int value = 1) {
		//用于初始化信号量，在使用信号量前必须这样做
		union USemun sem_union;
		sem_union.val = value;

		int bkd_value = BKDRHash(sem_topic.c_str());

		int sem_id = semget((key_t)bkd_value, 1, 0666 | IPC_CREAT );
		if (semctl(sem_id, 0, SETVAL, sem_union) == -1) {
			std::cout << "SETVAL errror" << std::endl;
			return -1;
		}
		return sem_id;
	}
	static void DelSemvalue(int sem_id) {
		//删除信号量
		union USemun sem_union;

		if (semctl(sem_id, 0, IPC_RMID, sem_union) == -1) {
			std::cout << "Failed to delete semaphore" << std::endl;
		}
	}

	static int GetSemvalue(int sem_id) {
		//get信号量
		union USemun sem_union;

		int value = semctl(sem_id, 0, GETVAL, sem_union);
		if (value == -1) {
			std::cout << "Failed to GetSemvalue semaphore" << std::endl;
		}
		return value;

	}
	static int Preprocess(int sem_id) {
		// std::cout << "GetSemvalue(sem_id):" << GetSemvalue(sem_id) << std::endl;
		while (GetSemvalue(sem_id) > 1) {
			union USemun sem_union;
			sem_union.val = 1;

			if (semctl(sem_id, 0, SETVAL, sem_union) == -1) {
				return -1;
			}
		}
		// std::cout << "after GetSemvalue(sem_id):" << GetSemvalue(sem_id) << std::endl;
		return 0;
	}

	static int Semaphore_p(int sem_id) {
		//对信号量做减1操作，即等待P（sv）
		struct sembuf sem_b;
		sem_b.sem_num = 0;
		sem_b.sem_op = -1;//P()
		sem_b.sem_flg = 0;
		// extern int errno;
		if (semop(sem_id, &sem_b, 1) == -1)
		{
			std::cout << "semaphore_p failed " << errno << std::endl;
			return -1;
		}
		return 0;
	}
	static int Semaphore_v(int sem_id) {
		struct sembuf sem_b;
		sem_b.sem_num = 0;
		sem_b.sem_op = 1;//V()
		sem_b.sem_flg = 0;
		// extern int errno;
		if (semop(sem_id, &sem_b, 1) == -1)
		{
			std::cout << "semaphore_v failed " << errno << std::endl;
			return -1;
		}
		return 0;
	}

};


}
}



#endif

