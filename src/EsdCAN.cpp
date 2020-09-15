#include "../include/can_bus/EsdCAN.hpp"
#include <iostream>

namespace can_bus{

EsdCAN::EsdCAN(int32_t port)
{
    port_ = port; 
    if(!init())
        std::cout<<" start can fault"<<std::endl;
    std::cout<<"can channel init ok"<<std::endl;
}
EsdCAN::~EsdCAN()
{
    if(dev_handler_)
        stop();
}
bool EsdCAN::init()
{
    //打开设备
    //std::cout<<port_<<std::endl;
    uint32_t mode = 0;
    if(port_<0||port_>3)
    {
        std::cout<<"can port is forbidden"<<std::endl;
        return false;
    }
    int32_t ret = canOpen(port_, mode, 2047,
                    2047, 0, 0, &dev_handler_);
    if (ret != NTCAN_SUCCESS)
    {
        std::cout<<"open device error code [" << ret << "]: " << GetErrorString(ret)<<std::endl;
        return false; 
    }
    int32_t id_count = 0x800;
    ret = canIdRegionAdd(dev_handler_, 0, &id_count);
    if (ret != NTCAN_SUCCESS)
    {
        std::cout<<"add receive msg id filter error code:" << ret << ", " << GetErrorString(ret)<<std::endl;
        return false;
    }
    ret = canSetBaudrate(dev_handler_, NTCAN_BAUD_500);
    if (ret != NTCAN_SUCCESS) 
    {
        std::cout << "set baudrate error code: " << ret << ", " << GetErrorString(ret);
        return false;
    }
    is_start_ = true;
    return true;
}
bool EsdCAN::Send(const std::vector<CanFrame> &frames, int32_t *const frame_num)
{
    if(frame_num == NULL)
        return false;
    if(frames.size() != static_cast<size_t>(*frame_num))
        return false;
    if(!is_start_)
    {
        std::cout<<"Esd CAN is not start"<<std::endl;
        return false;
    }
    for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
        send_frames_[i].id = frames[i].id;
        send_frames_[i].len = frames[i].len;
        std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);
    }
    
    int32_t ret = canWrite(dev_handler_, send_frames_, frame_num, nullptr);
    if (ret != NTCAN_SUCCESS) {
        std::cout << "send message failed, error code: " << ret << ", " << GetErrorString(ret);
        return false;
    }
    return true;
}
bool EsdCAN::Receive(std::vector<CanFrame> *const frames,int32_t *const frame_num)
{
    if(!is_start_)
    {
        std::cout<<"Esd CAN is not start"<<std::endl;
        return false;
    }
    if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
        std::cout << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
       << "], frame_num:" << *frame_num;
        return false;
    }
    //std::cout<<"This is true?"<<std::endl;
    const int32_t ret = canRead(dev_handler_, recv_frames_, frame_num, nullptr);
    if (ret == NTCAN_RX_TIMEOUT) {
        return true;
    }
    if (ret != NTCAN_SUCCESS) {
        std::cout << "receive message failed, error code: " << ret << ", "<< GetErrorString(ret);
        return false;
    }

    for (int32_t i = 0; i < *frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
        CanFrame cf;
        cf.id = recv_frames_[i].id;
        cf.len = recv_frames_[i].len;
        std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].len);
        frames->push_back(cf);
    }
    return true;
}
void EsdCAN::stop()
{
    if(is_start_)
    {
        is_start_ = false;
        int32_t ret = canClose(dev_handler_);
        if (ret != NTCAN_SUCCESS) 
        {
            std::cout << "close error code:" << ret << ", " << GetErrorString(ret);
        } else {
            std::cout << "close esd can ok. port:" << port_;
        }
    }
}

std::string EsdCAN::GetErrorString(const NTCAN_RESULT ntstatus) 
{
    struct ERR2STR 
    {
        NTCAN_RESULT ntstatus;
        const char *str;
    };
    int8_t str_buf[ERROR_BUF_SIZE];
    static const struct ERR2STR err2str[] = {
        {NTCAN_SUCCESS, "NTCAN_SUCCESS"},
        {NTCAN_RX_TIMEOUT, "NTCAN_RX_TIMEOUT"},
        {NTCAN_TX_TIMEOUT, "NTCAN_TX_TIMEOUT"},
        {NTCAN_TX_ERROR, "NTCAN_TX_ERROR"},
        {NTCAN_CONTR_OFF_BUS, "NTCAN_CONTR_OFF_BUS"},
        {NTCAN_CONTR_BUSY, "NTCAN_CONTR_BUSY"},
        {NTCAN_CONTR_WARN, "NTCAN_CONTR_WARN"},
        {NTCAN_NO_ID_ENABLED, "NTCAN_NO_ID_ENABLED"},
        {NTCAN_ID_ALREADY_ENABLED, "NTCAN_ID_ALREADY_ENABLED"},
        {NTCAN_ID_NOT_ENABLED, "NTCAN_ID_NOT_ENABLED"},
        {NTCAN_INVALID_FIRMWARE, "NTCAN_INVALID_FIRMWARE"},
        {NTCAN_MESSAGE_LOST, "NTCAN_MESSAGE_LOST"},
        {NTCAN_INVALID_PARAMETER, "NTCAN_INVALID_PARAMETER"},
        {NTCAN_INVALID_HANDLE, "NTCAN_INVALID_HANDLE"},
        {NTCAN_NET_NOT_FOUND, "NTCAN_NET_NOT_FOUND"},
    #ifdef NTCAN_IO_INCOMPLETE
        {NTCAN_IO_INCOMPLETE, "NTCAN_IO_INCOMPLETE"},
    #endif
    #ifdef NTCAN_IO_PENDING
        {NTCAN_IO_PENDING, "NTCAN_IO_PENDING"},
    #endif
    #ifdef NTCAN_INVALID_HARDWARE
        {NTCAN_INVALID_HARDWARE, "NTCAN_INVALID_HARDWARE"},
    #endif
    #ifdef NTCAN_PENDING_WRITE
        {NTCAN_PENDING_WRITE, "NTCAN_PENDING_WRITE"},
    #endif
    #ifdef NTCAN_PENDING_READ
        {NTCAN_PENDING_READ, "NTCAN_PENDING_READ"},
    #endif
    #ifdef NTCAN_INVALID_DRIVER
        {NTCAN_INVALID_DRIVER, "NTCAN_INVALID_DRIVER"},
    #endif
    #ifdef NTCAN_OPERATION_ABORTED
        {NTCAN_OPERATION_ABORTED, "NTCAN_OPERATION_ABORTED"},
    #endif
    #ifdef NTCAN_WRONG_DEVICE_STATE
        {NTCAN_WRONG_DEVICE_STATE, "NTCAN_WRONG_DEVICE_STATE"},
    #endif
        {NTCAN_INSUFFICIENT_RESOURCES, "NTCAN_INSUFFICIENT_RESOURCES"},
    #ifdef NTCAN_HANDLE_FORCED_CLOSE
        {NTCAN_HANDLE_FORCED_CLOSE, "NTCAN_HANDLE_FORCED_CLOSE"},
    #endif
    #ifdef NTCAN_NOT_IMPLEMENTED
        {NTCAN_NOT_IMPLEMENTED, "NTCAN_NOT_IMPLEMENTED"},
    #endif
    #ifdef NTCAN_NOT_SUPPORTED
        {NTCAN_NOT_SUPPORTED, "NTCAN_NOT_SUPPORTED"},
    #endif
    #ifdef NTCAN_SOCK_CONN_TIMEOUT
        {NTCAN_SOCK_CONN_TIMEOUT, "NTCAN_SOCK_CONN_TIMEOUT"},
    #endif
    #ifdef NTCAN_SOCK_CMD_TIMEOUT
        {NTCAN_SOCK_CMD_TIMEOUT, "NTCAN_SOCK_CMD_TIMEOUT"},
    #endif
    #ifdef NTCAN_SOCK_HOST_NOT_FOUND
        {NTCAN_SOCK_HOST_NOT_FOUND, "NTCAN_SOCK_HOST_NOT_FOUND"},
    #endif
    #ifdef NTCAN_CONTR_ERR_PASSIVE
        {NTCAN_CONTR_ERR_PASSIVE, "NTCAN_CONTR_ERR_PASSIVE"},
    #endif
    #ifdef NTCAN_ERROR_NO_BAUDRATE
        {NTCAN_ERROR_NO_BAUDRATE, "NTCAN_ERROR_NO_BAUDRATE"},
    #endif
    #ifdef NTCAN_ERROR_LOM
        {NTCAN_ERROR_LOM, "NTCAN_ERROR_LOM"},
    #endif
        {(NTCAN_RESULT)0xffffffff, "NTCAN_UNKNOWN"} /* stop-mark */
    };
    const struct ERR2STR *es = err2str;
    do {
        if (es->ntstatus == ntstatus) {
        break;
        }
        es++;
    } while ((uint32_t)es->ntstatus != 0xffffffff);
    #ifdef NTCAN_ERROR_FORMAT_LONG
    {
        NTCAN_RESULT res;
        char sz_error_text[60];
        res = canFormatError(ntstatus, NTCAN_ERROR_FORMAT_LONG, sz_error_text,
                            sizeof(sz_error_text) - 1);
        if (NTCAN_SUCCESS == res) {
        snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s - %s",
                es->str, sz_error_text);
        } else {
        snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
                es->str, ntstatus);
        }
    }
    #else
    snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
            es->str, ntstatus);
    #endif /* of NTCAN_ERROR_FORMAT_LONG */
    return std::string((const char *)(str_buf));
}

}