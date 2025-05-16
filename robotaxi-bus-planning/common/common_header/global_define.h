/*************************************************************************
    > File Name: globaldefine.h
    > Author:
    > Mail:
    > Created Time: 2018年03月04日 星期日 17时21分20秒
 ************************************************************************/

#ifndef _GLOBALDEFINE_H__
#define _GLOBALDEFINE_H__

#include <sys/syscall.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sched.h>

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;


#define GetTid() syscall(__NR_gettid)
#define GetCoreNum sysconf(_SC_NPROCESSORS_CONF);
#define gettidv2() syscall(SYS_gettid)


#ifndef BASE_RESET
#define BASE_RESET   "\033[0m"
#endif

#ifndef BASE_BLACK
#define BASE_BLACK   "\033[30m"      /* Black */
#endif

#ifndef BASE_RED
#define BASE_RED     "\033[31m"      /* Red */
#endif

#ifndef BASE_GREEN
#define BASE_GREEN   "\033[32m"      /* Green */
#endif

#ifndef BASE_YELLOW
#define BASE_YELLOW  "\033[33m"      /* Yellow */
#endif

#ifndef BASE_BLUE
#define BASE_BLUE    "\033[34m"      /* Blue */
#endif

#ifndef BASE_MAGENTA
#define BASE_MAGENTA "\033[35m"      /* Magenta */
#endif

#ifndef BASE_CYAN
#define BASE_CYAN    "\033[36m"      /* Cyan */
#endif

#ifndef BASE_WHITE
#define BASE_WHITE   "\033[37m"      /* White */
#endif

#ifndef BASE_BOLDBLACK
#define BASE_BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#endif

#ifndef BASE_BOLDRED
#define BASE_BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#endif

#ifndef BASE_BOLDGREEN
#define BASE_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#endif

#ifndef BASE_BOLDYELLOW
#define BASE_BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#endif

#ifndef BASE_BOLDBLUE
#define BASE_BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#endif

#ifndef BASE_BOLDMAGENTA
#define BASE_BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#endif

#ifndef BASE_BOLDCYAN
#define BASE_BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#endif

#ifndef BASE_BOLDWHITE
#define BASE_BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
#endif

typedef struct TFData
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    TFData() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
    }
} TFData;

#endif
