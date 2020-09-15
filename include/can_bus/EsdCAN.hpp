#pragma once

#include "ntcan.h"
#include <string>
#include <vector>
#include <cstring>


const int32_t MAX_CAN_SEND_FRAME_LEN = 10;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;

namespace can_bus{

struct CanFrame {
    /// Message id
    uint32_t id;
    /// Message length
    uint8_t len;
    /// Message content
    uint8_t data[8];
    /// Time stamp
    struct timeval timestamp;

    /**
     * @brief Constructor
     */
    CanFrame() : id(0), len(0), timestamp{0} {
    std::memset(data, 0, sizeof(data));
    };
};

class EsdCAN{

public:
    EsdCAN(int32_t port);
    ~EsdCAN();
    bool Receive(std::vector<CanFrame> *const frames,int32_t *const frame_num);
    bool Send(const std::vector<CanFrame> &frames,int32_t *const frame_num);
    bool init();
    
private:
    int32_t port_;
    const int32_t ERROR_BUF_SIZE = 200;
    bool is_start_ = false;
    CMSG send_frames_[MAX_CAN_SEND_FRAME_LEN];
    CMSG recv_frames_[MAX_CAN_RECV_FRAME_LEN];
    NTCAN_HANDLE dev_handler_;
    int seq_ = 0;
    
    
    void stop();
    void Parse(uint8_t leng,uint32_t uid,const uint8_t* data);
    
    std::string GetErrorString(const NTCAN_RESULT ntstatus);
};
}