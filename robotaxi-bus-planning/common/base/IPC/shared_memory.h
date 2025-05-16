#ifndef __SHARED_MEMORY_H__
#define __SHARED_MEMORY_H__

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "errno.h"
#include <string.h>
#include <atomic>
#include <iostream>

namespace acu {
namespace common {

typedef struct AreaLocation
{
	int array_index;
	long long seq_num;
	AreaLocation() {
		array_index = -1;
		seq_num = -1;
	}
} AreaLocation;


enum class eShmAreaStatus { IDLE = 0, WRITING, READING, UPDATED };
typedef struct ShmHeader
{
	std::atomic<int> shm_attach_times;
	std::atomic<int> buffer_len;
	AreaLocation lowest_loc;
	AreaLocation highest_loc;
	ShmHeader() {
		shm_attach_times.store(0);
		buffer_len.store(-1);
	}
	void PrintInfo() {
		std::cout << "shm info : " << shm_attach_times.load() << std::endl;
		std::cout << "lowest_loc:" << lowest_loc.array_index << "|" << lowest_loc.seq_num << std::endl;
		std::cout << "highest_loc:" << highest_loc.array_index << "|" << highest_loc.seq_num << std::endl;
	}
} ShmHeader;



typedef struct ShmAreaHeader
{
	std::atomic<eShmAreaStatus> shm_status;
	std::atomic<long long> sequence;
	std::atomic<double> send_time_stamp;
	ShmAreaHeader() {
		shm_status.store(eShmAreaStatus::IDLE);
		sequence.store(0);
		send_time_stamp.store(0.0);
	}
} ShmAreaHeader, *pShmAreaHeader;


typedef struct ShmInfo
{
	int shm_nattach;
	long last_atttime;
	void print() {
		std::cout << "shm_nattach:" << shm_nattach << std::endl;
	}
} ShmInfo;

class SharedMemory
{
public:
	SharedMemory() {}
	~SharedMemory() {}

	static unsigned int BKDRHash(const char *str)
	{
		unsigned int seed = 131; // 31 131 1313 13131 131313 etc..
		unsigned int hash = 0;

		while (*str)
		{
			hash = hash * seed + (*str++);
		}

		return (hash & 0x7FFFFFFF);
	}
	static int sharedMemoryGet(key_t key, int size) {
		int shmid;
		if ((shmid = shmget(key, size, 0666)) == -1)
			return -1;

		return shmid;
	}

	static int sharedMemoryCreate(key_t key, int size)
	{
		int shmid;

		if ((shmid = shmget(key, size, IPC_CREAT | 0666)) == -1)
			return -1;

		return shmid;
	}

	static int sharedMemoryCreateOrGet(key_t key, int size)
	{
		int shmid;

		/* If memory has already been created.. then just get it: */
		if ((shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | 0666)) == -1)
		{
			if ( errno != EEXIST )
				return -1;

			if ((shmid = shmget(key, size, 0666)) == -1)
				return -1;
		}

		return shmid;
	}

	static int sharedMemoryCreateIfGone(key_t key, int size)
	{
		int shmid;

		/* If memory has already been created.. then just get it: */
		if ((shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | 0666)) == -1)
			return errno;

		return shmid;
	}

	static void* sharedMemoryAttach(int shmid)
	{
		return shmat(shmid, 0, 0);
	}

	static int sharedMemoryDetatch(const void* shmaddr)
	{
		if ( shmdt(shmaddr) == -1 )
			return errno;

		return 0;
	}

	static int sharedMemoryInfo(int shmid, ShmInfo &info) {
		struct shmid_ds ds;
		if ( shmctl(shmid, IPC_STAT, &ds) == -1 )
			return errno;
		info.shm_nattach = ds.shm_nattch;
		info.last_atttime = ds.shm_atime;
		return 0;
	}

	static int sharedMemoryDelete(int shmid)
	{
		if ( shmctl(shmid, IPC_RMID, 0) == -1 )
			return errno;

		return 0;
	}

	static int sharedMemoryLock(int shmid)
	{
		if ( shmctl(shmid, SHM_LOCK, 0) == -1 )
			return errno;

		return 0;
	}

	static int sharedMemoryUnlock(int shmid)
	{
		if ( shmctl(shmid, SHM_UNLOCK, 0) == -1 )
			return errno;

		return 0;
	}

	static int ResizeSharedMemory(const int &shmid, const int &key, const int &size_bak, const int &target_size) {
		char *bak_data = new char[size_bak];
		char *from_mem = (char *)sharedMemoryAttach(shmid);
		memcpy(bak_data, from_mem, size_bak);
		sharedMemoryDelete(shmid);
		int target_shmid = sharedMemoryCreateOrGet(key, target_size);
		char *to_mem = (char *)sharedMemoryAttach(target_shmid);
		memcpy(to_mem, bak_data, size_bak);
		delete[] bak_data;
		return target_shmid;

	}


};

}
}


#endif

