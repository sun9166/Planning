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
// #pragma pack(4)
	typedef struct AreaLocation
	{
		int array_index;
		long long seq_num;
		AreaLocation()
		{
			reset();
		}
		void reset()
		{
			array_index = -1;
			seq_num = -1;
		}
	} AreaLocation;

	enum class eShmAreaStatus
	{
		IDLE = 0,
		WRITING,
		READING,
		UPDATED
	};
	typedef struct ShmHeader
	{
		std::atomic<int> pub_seq;
		std::atomic<int> data_seq;
		std::atomic<int> shm_attach_times;
		std::atomic<int> buffer_len;
		std::atomic<int> fixd_msg_data_size;
		std::atomic<int> StartPub;
		std::atomic<int> StartSub;
		std::atomic<int> DataShmNeedChanged;

		union UBitOpt
		{
			struct SBitOpt
			{
				unsigned int bStartPub : 1;			   // pub计数器，0~1
				unsigned int bStartSub : 5;			   // sub计数器(编号)，0~31
				unsigned int bDataShmNeedChanged : 32; //数据区域改变则对应的sub计数器里的编号位置1，
													   // sub收到后会重新获取内存再置0,大支持32个sub
				unsigned int Recoverd : 26;			   //最大支持32个sub
			} sBitOpt;
			// unsigned int u32BitOpt[2];
		} bit_opt;
		AreaLocation lowest_loc;
		AreaLocation highest_loc;
		ShmHeader()
		{
			shm_attach_times.store(0);
			buffer_len.store(-1);
		}
		void PrintInfo()
		{
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
		std::atomic<int> real_msg_data_size;
		std::atomic<int> check_code;
		ShmAreaHeader()
		{
			shm_status.store(eShmAreaStatus::IDLE);
			sequence.store(0);
			send_time_stamp.store(0.0);
		}
	} ShmAreaHeader, *pShmAreaHeader;

	typedef struct ShmInfo
	{
		int shm_nattach;
		long last_atttime;
		void print()
		{
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
		static int sharedMemoryGet(key_t key, int size)
		{
			return shmget(key, size, 0666);
		}

		static int sharedMemoryCreate(key_t key, int size)
		{
			int shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | 0666);
			if (shmid != -1)
			{
				// std::cout << "create shm" << std::endl;
			}
			else if (errno == EEXIST)
			{
				shmid = shmget(key, 0, 0666);
				if (shmid != -1)
				{
					struct shmid_ds buf;
					shmctl(shmid, IPC_STAT, &buf);
					if (buf.shm_segsz < size)
					{
						// std::cout << "shm exist need size = " << size << ", current size = " << buf.shm_segsz << std::endl;
						sharedMemoryDelete(shmid);
						shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | 0666);
					}
					else
					{
						// std::cout << "shm exist" << std::endl;
						shmid = shmget(key, buf.shm_segsz, 0666);
					}
				}
			}
			if (shmid == -1)
			{
				std::cout << "shmid = -1, errno = " << errno << std::endl;
			}
			else
			{
				char *to_mem = (char *)sharedMemoryAttach(shmid);
				memset(to_mem, 0, size);
				// std::cout << "create shm sucess key = " << key << ", size = " << size << std::endl;
			}
			return shmid;
		}

		static int sharedMemoryCreateOrGet(key_t key, int size)
		{
			bool bCreate;
			return SharedMemory::sharedMemoryCreateOrGet(key, size, bCreate);
		}

		static int sharedMemoryCreateOrGet(key_t key, int size, bool &bCreate)
		{
			bCreate = false;
			int shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | 0666);
			if (shmid != -1)
			{
				// std::cout << "create shm" << std::endl;
				char *to_mem = (char *)sharedMemoryAttach(shmid);
				memset(to_mem, 0, size);
				bCreate = true;
			}
			else if (errno == EEXIST)
			{
				shmid = shmget(key, 0, 0666);
				if (shmid != -1)
				{
					struct shmid_ds buf;
					shmctl(shmid, IPC_STAT, &buf);
					if (buf.shm_segsz < size)
					{
						shmid = -1;
						// std::cout << "shm exist need size = " << size << ", current size = " << buf.shm_segsz << std::endl;
					}
					else
					{
						// std::cout << "shm exist" << std::endl;
						shmid = shmget(key, buf.shm_segsz, 0666);
					}
				}
			}
			if (shmid == -1)
			{
				std::cout << "shmid = -1, errno = " << errno << std::endl;
			}
			else
			{
				// std::cout << "get or create shm sucess key = " << key << ", size = " << size << std::endl;
			}
			return shmid;
		}

		static int sharedMemoryCreateIfGone(key_t key, int size)
		{
			int shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | 0666);

			/* If memory has already been created.. then just get it: */
			if (shmid == -1)
				return errno;

			return shmid;
		}

		static void *sharedMemoryAttach(int shmid)
		{
			return shmat(shmid, 0, 0);
		}

		static int sharedMemoryDetatch(const void *shmaddr)
		{
			if (shmdt(shmaddr) == -1)
				return errno;

			return 0;
		}

		static int sharedMemoryInfo(int shmid, ShmInfo &info)
		{
			struct shmid_ds ds;
			if (shmctl(shmid, IPC_STAT, &ds) == -1)
				return errno;
			info.shm_nattach = ds.shm_nattch;
			info.last_atttime = ds.shm_atime;
			return 0;
		}

		static int sharedMemoryDelete(int shmid)
		{
			if (shmctl(shmid, IPC_RMID, 0) == -1)
			{
				return errno;
			}

			return 0;
		}

		static int sharedMemoryLock(int shmid)
		{
			if (shmctl(shmid, SHM_LOCK, 0) == -1)
				return errno;

			return 0;
		}

		static int sharedMemoryUnlock(int shmid)
		{
			if (shmctl(shmid, SHM_UNLOCK, 0) == -1)
				return errno;

			return 0;
		}

		static int ResizeSharedMemory(const int &shmid, const int &key, const int &size_bak, const int &target_size)
		{
			return SharedMemory::ResizeSharedMemory(shmid, key, size_bak, target_size, 1, 0, size_bak);
			// memcpy(to_mem, bak_data, size_bak);
		}

		static int ResizeSharedMemory(const int &shmid, const int &key, const int &size_bak, const int &target_size,
									  const int buf_len, const int buf_size, const int buf_size_bak)
		{
			char *bak_data = new char[size_bak];
			char *from_mem = (char *)sharedMemoryAttach(shmid);
			memcpy(bak_data, from_mem, size_bak);
			sharedMemoryDelete(shmid);
			int target_shmid = sharedMemoryCreate(key, target_size);
			char *to_mem = (char *)sharedMemoryAttach(target_shmid);
			for (int i = 0; i < buf_len; i++)
			{
				memcpy(to_mem + i * buf_size, bak_data + i * buf_size_bak, buf_size_bak);
			}

			delete[] bak_data;
			return target_shmid;
		}
	};
// #pragma pack()
}
}


#endif

