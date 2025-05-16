#ifndef __COMMON_WATCHDOG_HANDLER_H__
#define __COMMON_WATCHDOG_HANDLER_H__

#include <string>
#include <vector>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>

#define	ACU_WATCHDOG_IOCTL_BASE	'I'
#define IMMEDIATE_TRIGGER_ID 234

#define GUARDIAN_PROCESS_NAME "Guardian"
#define GUARD_SIGNAL_COMMON 56
#define GUARD_SIGNAL_KILLED 57
#define GUARD_SIGNAL_CREATE 58

struct acu_node_info {
	int pid;
	int timeout;
	int status;
	unsigned long long last_keepalive;
	unsigned long long jiffies;
	char name[32];
};

#define	ACU_WDIOC_ADDNODE	       _IOWR(ACU_WATCHDOG_IOCTL_BASE, 0, struct acu_node_info)
#define	ACU_WDIOC_GETFAILNODE     _IOWR(ACU_WATCHDOG_IOCTL_BASE, 1, struct acu_node_info)
#define	ACU_WDIOC_KEEPALIVE	   _IOWR(ACU_WATCHDOG_IOCTL_BASE, 2, int)
#define	ACU_WDIOC_SETGUARD	       _IOWR(ACU_WATCHDOG_IOCTL_BASE, 3, int)

namespace acu {
namespace common {
namespace watchdog {

class WatchdogHandler
{
private:
	int watchdog_fd_;
	struct acu_node_info info;

public:
	WatchdogHandler() {
		watchdog_fd_ = -1;
		OpenDog();
	};
	~WatchdogHandler()
	{
		if (watchdog_fd_ != -1) {
			close(watchdog_fd_);
			watchdog_fd_ = -1;
		}
	};

	int RegistNode(int timeout, char *name) {
		info.pid = getpid();
		info.timeout = timeout;
		strncpy(info.name, name, sizeof(info.name));

		if (0 != OpenDog()) {
			return -1;
		}

		if (ioctl(watchdog_fd_, ACU_WDIOC_ADDNODE, &info) < 0) {
			fprintf(stderr, "Watchdog device set info fail!.\n");
			fflush(stderr);
			close(watchdog_fd_);
			watchdog_fd_ = -1;
		}
		return 0;
	}

	int FeedDog() {
		int pid;
		if (watchdog_fd_ == -1) return -1;
		pid = getpid();
		if (ioctl(watchdog_fd_, ACU_WDIOC_KEEPALIVE, &pid) < 0) {
			fprintf(stderr, "Watchdog feed error!.\n");
			fflush(stderr);
			close(watchdog_fd_);
			watchdog_fd_ = -1;
			return -1;
		}
		return 0;
	}

private:
    int OpenDog()
    {
        if (watchdog_fd_ == -1) {
            watchdog_fd_ = open("/dev/acu_watchdog", O_WRONLY);
            if (watchdog_fd_ == -1) {
                fprintf(stderr, "Watchdog device not enabled.\n");
                fflush(stderr);
                return -1;
            }
        }
        return 0;
    }
};
}
}
}

#endif




