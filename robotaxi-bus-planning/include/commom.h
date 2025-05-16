#ifndef _COMMOM_H_
  #define _COMMOM_H_
#include <string>

#define NUM_MAX 10

using namespace std; 

class COriginalBytes
{
public:
    COriginalBytes():
        m_data(nullptr),m_size(0)
    {

    }
    COriginalBytes(void* data_ptr,unsigned int data_size):
        m_data(data_ptr),m_size(data_size)
    {

    }
    virtual ~COriginalBytes() {}
    void set_data(void* data_ptr,unsigned int data_size)
    {
        m_data = data_ptr;
        m_size = data_size;
    }

    void set_data(void* data_ptr,unsigned int data_size,unsigned long long write_stamp)
    {
        m_data = data_ptr;
        m_size = data_size;
        m_write_stamp = write_stamp;
    }

    void* data()
    {
        return m_data;
    }

    unsigned int data_size()
    {
        return m_size;
    }

    unsigned long long write_stamp()
    {
        return m_write_stamp;
    }
    unsigned int index()
    {
        return m_index;
    }

    const std::string topic()
    {
        return m_topic;
    }

public:
    void* m_data;
    unsigned int m_size;
    unsigned int m_index;
    unsigned long long m_write_stamp;
    std::string m_topic;
};


typedef struct t_topic_dds
{
    string type;
    string topic_name;
    string topic_fuction;
    int topic_period;
    string topic_netdev;
}t_topic_dds;

typedef struct t_topic_zmq
{
    string type;
    string topic_name;
    string topic_fuction;
    int topic_period;
    string topic_target;
    int topic_port;
}t_topic_zmq;

typedef struct t_topic_shm
{
    string type;
    string topic_name;
    string topic_fuction;
    int topic_period;
}t_topic_shm;

extern int dds_sub_num;
extern int dds_pub_num;
extern int zmq_rep_num;
extern int zmq_req_num;
extern int zmq_pub_num;
extern int zmq_sub_num;
extern int shm_sub_num;
extern int shm_pub_num;

extern string node_name;

extern t_topic_dds t_dds_sub[NUM_MAX];
extern t_topic_dds t_dds_pub[NUM_MAX];
extern t_topic_zmq t_zmq_rep[NUM_MAX];
extern t_topic_zmq t_zmq_req[NUM_MAX];
extern t_topic_zmq t_zmq_pub[NUM_MAX];
extern t_topic_zmq t_zmq_sub[NUM_MAX];
extern t_topic_shm t_shm_sub[NUM_MAX];
extern t_topic_shm t_shm_pub[NUM_MAX];


#endif