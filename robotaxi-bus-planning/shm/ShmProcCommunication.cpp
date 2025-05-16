#include "ShmProcCommunication.h"


namespace acu {
namespace shm {


ShmProcCommunication::ShmProcCommunication() {
        header_buffer_pointer = nullptr;
        topic = "";
        shmid_header_ = -1;
        is_data_empty = true;
        std::vector<std::string> res_vec;
        ExeLinuxCmd(
            "ipcs -m | awk '{if($6==0) print $2,$4}' | "
            "grep 666 | awk '{print $1}' "
            "| xargs -I {} ipcrm -m {}",
            res_vec);
}

ShmProcCommunication::~ShmProcCommunication() {
        // std::cout << "Uninit ShmProcCommunication" << std::endl;
        ShmInfo info;
        if (shmid_header_ != -1) {
            acu::common::SharedMemory::sharedMemoryDetatch((void *)header_buffer_pointer);
            acu::common::SharedMemory::sharedMemoryInfo(shmid_header_, info);
            if (info.shm_nattach == 0) {
                // std::cout << "delete" << std::endl;
                acu::common::SharedMemory::sharedMemoryDelete(shmid_header_);
            }
        }
        header_buffer_pointer = nullptr;

        if (shmid_data_ != -1) {
            acu::common::SharedMemory::sharedMemoryDetatch((void *)data_buffer_pointer);
            acu::common::SharedMemory::sharedMemoryInfo(shmid_data_, info);
            if (info.shm_nattach == 0) {
                // std::cout << "delete" << std::endl;
                acu::common::SharedMemory::sharedMemoryDelete(shmid_data_);
            }
        }
        data_buffer_pointer = nullptr;
}

int ShmProcCommunication::InitReader(const std::string &_topic) {
        // std::cout << "Init ShmProcCommunication InitReader" << std::endl;
        topic = _topic;
        data_topic = topic + "data";
        msg_data_topic = topic + "msg_data";

        key_t key = acu::common::SharedMemory::BKDRHash(topic.c_str());
        int size = (sizeof(ShmHeader) / 4096 + 1) * 4096;
        shmid_header_ = acu::common::SharedMemory::sharedMemoryCreateOrGet(key, size);
        // std::cout << "object_shmid_header_:" <<
        // shmid_header_ << std::endl;
        if (shmid_header_ == -1)
            return -1;
        header_buffer_pointer = (ShmHeader *)acu::common::SharedMemory::sharedMemoryAttach(shmid_header_);
        pub_seq = header_buffer_pointer->pub_seq.load();
        data_seq = header_buffer_pointer->data_seq.load();
        whitch_one_sub = -1;
        buffer_len = 10;

        return 0;
}

int ShmProcCommunication::InitWriter(const std::string &_topic) {
        // std::cout << "Init ShmProcCommunication
        // InitWriter" << std::endl;
        topic = _topic;
        data_topic = topic + "data";
        msg_data_topic = topic + "msg_data";
        // this->ClearSemaphore(); // chhTODO
        key_t key = acu::common::SharedMemory::BKDRHash(topic.c_str());
        int size = (sizeof(ShmHeader) / 4096 + 1) * 4096;
        shmid_header_ = acu::common::SharedMemory::sharedMemoryCreate(key, size);

        if (shmid_header_ == -1)
            return -1;
        // std::cout << "object_shmid_header_:" <<
        // shmid_header_ << std::endl;
        header_buffer_pointer = (ShmHeader *)acu::common::SharedMemory::sharedMemoryAttach(shmid_header_);

        srand(time(NULL));
        int pub_seq = (rand() % 10000 + 1);    //common::Util::rand_number();
        header_buffer_pointer->pub_seq.store(pub_seq);
        header_buffer_pointer->data_seq.store(0);
        header_buffer_pointer->buffer_len.store(10);
        header_buffer_pointer->fixd_msg_data_size.store(0);
        header_buffer_pointer->shm_attach_times++;
        header_buffer_pointer->bit_opt.sBitOpt.bDataShmNeedChanged = 0x0;
        buffer_len = header_buffer_pointer->buffer_len.load();
        return 0;
    }

bool ShmProcCommunication::InitDataBuffer(CCommonTopicMsg **data) {
        if (header_buffer_pointer == nullptr) {
            std::cout << "InitDataBuffer: header_buffer_pointer is null" << topic << std::endl;
            return false;
        }
        if (pub_seq != header_buffer_pointer->pub_seq.load()) {
            //std::cout << "InitDataBuffer: new pub" << topic << std::endl;
            read_loc.reset();
            pub_seq = header_buffer_pointer->pub_seq.load();
            data_seq = header_buffer_pointer->data_seq.load();
            whitch_one_sub = -1;
            buffer_len = 10;
            if (read_sempore_id != -1) {
                printf("GetSemvalue(%d) = %d\n",read_sempore_id,
                    acu::common::SemporeTransmitter::GetSemvalue(read_sempore_id));
            }
        }
        if (whitch_one_sub == -1 && data_seq == header_buffer_pointer->data_seq.load()) {
            return false;
        }
        if (whitch_one_sub == -1) {
            // std::cout << "..................chhdebug
            // InitDataBuffer: " << "whitch_one_sub == 0" <<
            // std::endl; chhTODO 锁一下
            whitch_one_sub = header_buffer_pointer->StartSub.load(std::memory_order_relaxed);

            //printf("whitch_one_sub=%x", whitch_one_sub);
            header_buffer_pointer->StartSub.fetch_add(1);
            //printf("header_buffer_pointer->StartSub=%x", header_buffer_pointer->StartSub.load(std::memory_order_relaxed));
            read_sempore_id = acu::common::SemporeTransmitter::InitSemvalue(topic + to_string(whitch_one_sub));
            // MAVOS_MIDDLEWARE_DEBUG("GetSemvalue(%d) =
            // %d\n", read_sempore_id,
            // avos::common::SemporeTransmitter::GetSemvalue(read_sempore_id));
            data_seq = header_buffer_pointer->data_seq.load();
            header_buffer_pointer->shm_attach_times++;
            
            msg_data_size = header_buffer_pointer->fixd_msg_data_size.load();
            shmid_data_ = acu::common::SharedMemory::sharedMemoryCreateOrGet(
                acu::common::SharedMemory::BKDRHash(data_topic.c_str()),
                ((msg_data_size + sizeof(ShmAreaHeader)) * buffer_len / 4096 + 1) * 4096);
            if (shmid_data_ == -1) {
                printf("InitDataBuffer: shmid_data_ == -1\n");
                return false;
            }
            // std::cout << "shmid_data_:" << shmid_data_ <<
            // std::endl;
            is_data_empty = false;
            data_buffer_pointer = (char *)acu::common::SharedMemory::sharedMemoryAttach(shmid_data_);
            *data = (CCommonTopicMsg *)malloc(msg_data_size);
        }
        if (data_buffer_pointer == nullptr) {
            printf(
                "InitDataBuffer: data_buffer_pointer is "
                "null\n");
            return false;
        }
        return true;
    }


bool ShmProcCommunication::GetData(CCommonTopicMsg **data) {
        if (!m_running || !InitDataBuffer(data)) {
            return false;
        }

        if (header_buffer_pointer->fixd_msg_data_size.load() > msg_data_size) {
            msg_data_size = header_buffer_pointer->fixd_msg_data_size.load();

            shmid_data_ = acu::common::SharedMemory::sharedMemoryCreateOrGet(
                acu::common::SharedMemory::BKDRHash(data_topic.c_str()),
                ((msg_data_size + sizeof(ShmAreaHeader)) * buffer_len / 4096 + 1) * 4096);
            if (shmid_data_ == -1)
                return -1;
            // std::cout << "shmid_data_:" << shmid_data_ <<
            // std::endl;
            is_data_empty = false;
            data_buffer_pointer = (char *)acu::common::SharedMemory::sharedMemoryAttach(shmid_data_);

            header_buffer_pointer->bit_opt.sBitOpt.bDataShmNeedChanged &= ~(1 << whitch_one_sub);

            free(*data);
            *data = (CCommonTopicMsg *)malloc(msg_data_size);
            read_loc.seq_num = -1;
        }

        // avos::common::SemporeTransmitter::Preprocess(read_sempore_id);
        // std::cout << "GetSemvalue(sem_id):" <<
        // GetSemvalue(sem_id) << std::endl;
       // printf(
       //     "GetSemvalue(%d) = %d\n", read_sempore_id, acu::common::SemporeTransmitter::GetSemvalue(read_sempore_id));
        if (acu::common::SemporeTransmitter::Semaphore_p(read_sempore_id) == -1) {
            printf("-------\n");
            return false;
        }
        if (!m_running) {
            printf("-------\n");
            return false;
        }
        printf("-------\n");
        pShmAreaHeader pointer_header;
        // std::cout << "read_loc " << read_loc.array_index
        // << "|" << read_loc.seq_num << std::endl;
        if (read_loc.seq_num == -1) {
            read_loc = header_buffer_pointer->highest_loc;
            pointer_header = GetIndexHeader((read_loc.array_index) % buffer_len);
            // avos::common::SemporeTransmitter::Semaphore_v(read_sempore_id);
        } else {
            int index = GetNextReadIndex(read_loc, pointer_header);
            if (index == -1)
                return false;
        }
        /*printf(
            "-------read_loc.array_index = "
            "%d,pointer_header->real_msg_data_size = "
            "%d,pointer_header->send_time_stamp = "
            "%f,pointer_header->check_code = "
            "0x%x\n",
            read_loc.array_index,
            pointer_header->real_msg_data_size.load(),
            pointer_header->send_time_stamp.load(),
            pointer_header->check_code.load());*/
        
        if (pointer_header->check_code.load() != POINT_HEADER_CHECK_CODE) {
            printf(
                "pointer_header->check_code.load() != "
                "0x20221223)\n");
            return false;
        }

        pointer_header->shm_status.store(eShmAreaStatus::READING);
        memcpy(
            (char *)(*data), (char *)pointer_header + sizeof(ShmAreaHeader), pointer_header->real_msg_data_size.load());

        // (*data)->topic_header.message_size =
        // pointer_header->real_msg_data_size.load();
        pointer_header->shm_status.store(eShmAreaStatus::IDLE);
        
        printf("------- topic data: %s\n", data_topic.c_str());
        return true;
}


bool ShmProcCommunication::SetData(const CCommonTopicMsg *data, std::shared_ptr<::google::protobuf::MessageLite> msg, int msg_len) {
        // std::cout << "msg_len " << msg_len << std::endl;
        if (header_buffer_pointer == nullptr) {
            return false;
        }
        
        //        if(msg_data_size_list.size()==header_buffer_pointer->buffer_len.load())
        if (msg_data_size_list.size() == 1) {
            UAcuCommunication64bitDataType pub_opt_tmp;
            pub_opt_tmp.opt = pub_opt;
            if (pub_opt_tmp.pub_opt.msg_mode_ == eMsgByteProtoMode) {
                msg_data_size = pub_opt_tmp.pub_opt.msg_max_size_;
                msg_data_size -= msg_data_size % 8;
            } else {
                double sum = accumulate(std::begin(msg_data_size_list), std::end(msg_data_size_list), 0.0);
                double mean = sum / msg_data_size_list.size();

                double variance = 0.0;
                for (uint16_t i = 0; i < msg_data_size_list.size(); i++) {
                    variance = variance + pow(msg_data_size_list[i] - mean, 2);
                }
                variance = variance / msg_data_size_list.size();

                if (variance == 0.0) {
                    //由于方差为0，为定长topic
                    msg_data_size = mean * 1.5;
                    msg_data_size -= msg_data_size % 8;
                } else {
                    //由于方差不为0，为不定长topic,为了性能不想随时动态大小内存申请
                    msg_data_size = mean * 1.5;
                    msg_data_size -= msg_data_size % 8;
                }
            }
            buffer_len = header_buffer_pointer->buffer_len.load();
            header_buffer_pointer->fixd_msg_data_size.store(msg_data_size);

            shmid_data_ = acu::common::SharedMemory::sharedMemoryCreate(
                acu::common::SharedMemory::BKDRHash(data_topic.c_str()),
                ((msg_data_size + sizeof(ShmAreaHeader)) * buffer_len / 4096 + 1) * 4096);
            //std::cout << "data_topic: " << data_topic<<std::endl;
            //std::cout << "msg_data_size: " << msg_data_size<<std::endl;
            //std::cout << "buffer_len: " << buffer_len<<std::endl;
            if (shmid_data_ == -1)
                return -1;
            //std::cout << "shmid_data_:" << shmid_data_ << std::endl;
            //            is_data_empty = false;
            data_buffer_pointer = (char *)acu::common::SharedMemory::sharedMemoryAttach(shmid_data_);
            msg_data_size_list.push_back(msg_len);

            

        }
        //        else
        //        if(msg_data_size_list.size()<header_buffer_pointer->buffer_len.load())
        else if (msg_data_size_list.size() < 1) {
            msg_data_size_list.push_back(msg_len);
        }

        if (msg_data_size == 0) {
            return false;
        }

        if (msg_len > msg_data_size) {
            //std::cout << "msg_len > msg_data_size"<< std::endl;
            int tmp_msg_data_size = msg_len * 1.5;
            tmp_msg_data_size -= tmp_msg_data_size % 8;

            shmid_data_ = acu::common::SharedMemory::ResizeSharedMemory(
                shmid_data_,
                acu::common::SharedMemory::BKDRHash(data_topic.c_str()),
                ((msg_data_size + sizeof(ShmAreaHeader)) * buffer_len / 4096 + 1) * 4096,
                ((tmp_msg_data_size + sizeof(ShmAreaHeader)) * buffer_len / 4096 + 1) * 4096,
                buffer_len,
                (tmp_msg_data_size + sizeof(ShmAreaHeader)),
                (msg_data_size + sizeof(ShmAreaHeader)));
            //std::cout << "data_topic: " << data_topic<< std::endl;
            //std::cout << "msg_data_size: " << msg_data_size<< std::endl;
            //std::cout << "buffer_len: " << buffer_len<< std::endl;
            //std::cout << "tmp_msg_data_size: " << tmp_msg_data_size<< std::endl;
            //printf("-------%s\n", data_topic.c_str());
            if (shmid_data_ == -1)
                return -1;

            msg_data_size = tmp_msg_data_size;
            buffer_len = header_buffer_pointer->buffer_len;
            header_buffer_pointer->fixd_msg_data_size.store(msg_data_size);

            for (int var = 0; var < header_buffer_pointer->StartSub.load(std::memory_order_relaxed); ++var) {
                header_buffer_pointer->bit_opt.sBitOpt.bDataShmNeedChanged |= (1 << var);
            }
            // std::cout << "shmid_data_:" << shmid_data_ <<
            // std::endl;
            data_buffer_pointer = (char *)acu::common::SharedMemory::sharedMemoryAttach(shmid_data_);
        }

        if (header_buffer_pointer == nullptr || data_buffer_pointer == nullptr) {
            return false;
        }

        pShmAreaHeader pointer_header;
        AreaLocation temp_highest_loc = header_buffer_pointer->highest_loc;
        // AINFO << "temp_highest_loc.array_index " <<
        // temp_highest_loc.array_index;
        int res = GetNextWriteIndex(temp_highest_loc, pointer_header);

        if (res == -1) {
            // pointer_header->shm_status.store(eShmAreaStatus::IDLE);
            return false;
        }
        pointer_header->shm_status.store(eShmAreaStatus::WRITING);

        auto now = std::chrono::system_clock::now();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());      //  huangchuo  get now time
        pointer_header->send_time_stamp.store(((double)microseconds.count())/1000000);
        header_buffer_pointer->highest_loc = temp_highest_loc;
        pointer_header->sequence = header_buffer_pointer->highest_loc.seq_num;
        pointer_header->real_msg_data_size.store(msg_len);
        pointer_header->check_code.store(POINT_HEADER_CHECK_CODE);

        // memcpy((char *)pointer_header +
        // sizeof(ShmAreaHeader), (char *)data,
        //        msg_len);
        memcpy((char *)pointer_header + sizeof(ShmAreaHeader), (char *)data, sizeof(CCommonTopicMsg::STopicMsgHeader));
        //printf("write_msg->topic_header.genstamp = %ld\n", (CCommonTopicMsg *)data->topic_header.genstamp);
        msg->SerializeToArray(
            (void *)((char *)pointer_header + sizeof(ShmAreaHeader) + sizeof(CCommonTopicMsg::STopicMsgHeader)),
            msg->ByteSize());
        /*printf(
            "TopicMsgHeader_size = %ld, SerializeToArray "
            "msg_size = %d\n",
            sizeof(CCommonTopicMsg::STopicMsgHeader),
            msg->ByteSize());*/
        FillInnerLoc(header_buffer_pointer->lowest_loc, header_buffer_pointer->highest_loc);
        pointer_header->shm_status.store(eShmAreaStatus::IDLE);
        /*printf(
            "header_buffer_pointer->highest_loc.array_"
            "index = "
            "%d,pointer_header->real_msg_data_size = "
            "%d,pointer_header->send_time_stamp = %f\n ",
            header_buffer_pointer->highest_loc.array_index,
            pointer_header->real_msg_data_size.load(),
            pointer_header->send_time_stamp.load());*/
        // std::cout << "..................chhdebug SetData
        // (all_sub_num): " << all_sub_num << std::endl;
        //多sub的信号量管理

        while (all_sub_num < header_buffer_pointer->StartSub.load(std::memory_order_relaxed)) {
            int werite_sempore_id = acu::common::SemporeTransmitter::InitSemvalue(topic + to_string(all_sub_num));
            werite_sempore_id_list.push_back(werite_sempore_id);
            all_sub_num++;
        }
        /*printf(
            "all_sub_num = "
            "%d,header_buffer_pointer->StartSub=%d\n",
            all_sub_num,
            header_buffer_pointer->StartSub.load(std::memory_order_relaxed));*/
        if (bfirstsetdata) {
            this->ClearSemaphore();
            bfirstsetdata = false;
        }
        /*printf(
            "all_sub_num = "
            "%d,header_buffer_pointer->StartSub=%d\n",
            all_sub_num,
            header_buffer_pointer->StartSub.load(std::memory_order_relaxed));*/
        if (bfirstsetdata) {
            this->ClearSemaphore();
            bfirstsetdata = false;
        }
        for (int var = 0; var < werite_sempore_id_list.size(); ++var) {
            /*printf(
                "GetSemvalue(%d) = %d\n",
                werite_sempore_id_list[var],
                acu::common::SemporeTransmitter::GetSemvalue(werite_sempore_id_list[var]));*/
            acu::common::SemporeTransmitter::Semaphore_v(werite_sempore_id_list[var]);
            // MAVOS_MIDDLEWARE_DEBUG("GetSemvalue(%d) =
            // %d\n", werite_sempore_id_list[var],
            // avos::common::SemporeTransmitter::GetSemvalue(werite_sempore_id_list[var]));
        }
        header_buffer_pointer->data_seq.store(header_buffer_pointer->data_seq.load() + 1);
        // std::cout << "..................chhdebug SetData
        // (header_buffer_pointer->data_seq.load()): " <<
        // header_buffer_pointer->data_seq.load() <<
        // std::endl;
        //
        //printf("-------\n");
        return true;
}


ShmAreaHeader* ShmProcCommunication::GetIndexHeader(const int &index) {
        if (index < 0 || index >= buffer_len) {
            return nullptr;
        }
        return (ShmAreaHeader *)((char *)data_buffer_pointer + index * (msg_data_size + sizeof(ShmAreaHeader)));
    }

int ShmProcCommunication::GetNextReadIndex(AreaLocation &loc, pShmAreaHeader &next_p) {
        for (int i = 1; i < buffer_len; i++) {
            ShmAreaHeader *p = GetIndexHeader((loc.array_index + i) % buffer_len);
            if (p == nullptr) {
                std::cout << "p is nullptr" << std::endl;
                return -1;
            }
            // std::cout << "AreaLocation loc:" <<
            // loc.seq_num << "|" << p->sequence
            // << std::endl;
            if (p->sequence > loc.seq_num && p->shm_status.load() != eShmAreaStatus::WRITING) {
                next_p = p;
                loc.array_index = (loc.array_index + i) % buffer_len;
                loc.seq_num = p->sequence;
                p->shm_status.store(eShmAreaStatus::READING);
                return 0;
            }
        }
        return -1;
}

int ShmProcCommunication::GetNextWriteIndex(AreaLocation &loc, pShmAreaHeader &area) {
        for (int i = 1; i < buffer_len; i++) {
            ShmAreaHeader *p = GetIndexHeader((loc.array_index + i) % buffer_len);
            if (p == nullptr) {
                std::cout << "p is nullptr" << std::endl;
                return -1;
            }
            int tmpa = (int)(p->shm_status.load());
            //printf("tmpa=%d\n", tmpa);
            if (p->shm_status.load() != eShmAreaStatus::READING) {
                loc.seq_num++;
                loc.array_index = (loc.array_index + i) % buffer_len;
                p->shm_status.store(eShmAreaStatus::WRITING);
                area = p;
                return 0;
            }
        }
        return -1;
}

void ShmProcCommunication::FillInnerLoc(AreaLocation &lowest, AreaLocation &highest) {
        int min_seq_num = 99999999;
        int max_seq_num = -1;

        AreaLocation temp_lowest, temp_highest;
        for (int i = 0; i < buffer_len; i++) {
            ShmAreaHeader *p = GetIndexHeader(i);
            if (p == nullptr) {
                std::cout << "p is nullptr" << std::endl;
            }
            if (p->sequence < min_seq_num) {
                min_seq_num = p->sequence;
                temp_lowest.array_index = i;
                temp_lowest.seq_num = min_seq_num;
            }
            if (p->sequence > max_seq_num) {
                max_seq_num = p->sequence;
                temp_highest.array_index = i;
                temp_highest.seq_num = max_seq_num;
            }
        }
        lowest = temp_lowest;
        highest = temp_highest;
}

int ShmProcCommunication::ExeLinuxCmd(const std::string &cmd, std::vector<std::string> &resvec) {
        resvec.clear();
        FILE *pp = popen(cmd.c_str(), "r");  //建立管道
        if (!pp) {
            return -1;
        }
        char tmp[1024];  //设置一个合适的长度，以存储每一行输出
        while (fgets(tmp, sizeof(tmp), pp) != NULL) {
            if (tmp[strlen(tmp) - 1] == '\n') {
                tmp[strlen(tmp) - 1] = '\0';  //去除换行符
            }
            resvec.push_back(tmp);
        }
        pclose(pp);  //关闭管道
        return resvec.size();
}

void ShmProcCommunication::ClearSemaphore() {
        for (int i = 0; i < 256; i++) {
            int id = acu::common::SemporeTransmitter::InitSemvalue(topic + to_string(i));
            if (id == -1) {
                continue;
            }
            acu::common::SemporeTransmitter::Semaphore_v(id);
            // avos::common::SemporeTransmitter::DelSemvalue(id);
        }
}



}
}