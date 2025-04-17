#ifndef CanHandler_H
#define CanHandler_H

#include <thread>
#include <mutex>
//tj #include "ICANCmd.h"
#include <ntcan.h>
#include <iostream>
#include <glog/logging.h>

#include <cstdint>
#include <stdexcept>
#include <cstring>
#include <vector>

// CanProcess命名空间， 使用 using namespace CanProcess; 或者 CanProcess::something
namespace CanProcess
{

    
    // 接受线程的参数， run默认值必须为 true， 否则将立即退出接受线程
    struct recv_args
    {
        bool run = true;
        unsigned int channel = 0;
    };
    extern recv_args recvArg;

    // 发送线程的参数， run同理
    
    struct send_args
    {
        bool run = true;
        std::mutex datalock; //
        CMSG *data = nullptr;
        unsigned int sndType = 2;
        unsigned int channel = 1;
        bool msg_arrived = false;
        int num_frames = 1;//一共定义了几个发送ID 必须和CanMsgDef.cc里对应？
        int interval = 20;
    };



    //  sendArg50 是间隔为50ms的线程， sendArg对应间隔为20ms的线程
    extern send_args sendArg;
    //tj extern send_args sendArg50;

    // 用于测试的线程锁
    //tj extern std::mutex testLock;
    //tj extern int test;

    class CanHandler
    {
    public:
        /**
         * Create a CAN processor
        */
        CanHandler();

        /**
         * Create a CAN processor
         * tj @param DevType 设备类型
         * tj @param Idex USB索引， 从0开始 tj
         * @param Channel 对应CAN卡上的CAN通道， 1代表 CAN0， 2代表 CAN1， 3代表CAN0和CAN1
         * tj @param config CAN卡的设定， 具体参考CanHandler.h 
        */
       //tj CanHandler(int DevType, int Idex, int Channel, CAN_InitConfig config);
        CanHandler(int Channel); // Channel为选择第几路CAN

        /**
         * 析构函数， 关闭Can通道
        */
        ~CanHandler()
        {
                int32_t ret = canClose(dev_handler_);
                if (ret != NTCAN_SUCCESS) {
                std::cout << "close error code:\n";} 
                else {
                std::cout << "close esd can ok. port\n";}
        }

        /**
         * 开启CAN卡设备， 并激活相应的通道 （参数是构造函数确定的）
        */
        unsigned int OpenDevice(int channel)
        {
            int port_=channel;

            uint32_t mode = 0;//应该默认就是0；拓展帧是0x10；

            int32_t ret = canOpen(port_, mode, NTCAN_MAX_TX_QUEUESIZE,
            NTCAN_MAX_RX_QUEUESIZE, 5, 5, &dev_handler_);
            if (ret != NTCAN_SUCCESS) {
             std::cout << "open device error code:\n";} 
            else {
            std::cout << "open device sussess:\n";}
            
            // 1. set receive message_id filter, ie white list 大概是有个CANid的滤波器，要把要的发过去
            int32_t id_count = 0x800;
            ret = canIdRegionAdd(dev_handler_, 0, &id_count);

            if (ret != NTCAN_SUCCESS) {
                std::cout << "add receive msg id filter error\n";}
            else {
                std::cout << "add receive msg id filter success\n";}

            // 2. set baudrate to 500k 波特率设置
            ret = canSetBaudrate(dev_handler_, NTCAN_BAUD_500);
            
            if (ret != NTCAN_SUCCESS) {
                std::cout << "set baudrate error\n";}
            else {
                std::cout << "set baudrate success\n";}
            sleep(1);
            if (ret != NTCAN_SUCCESS) {
                return(0);}
            else {
                return(1);}
            

        }

        /**
         * 接受数据的线程
         * @arg 接受数据的参数
        */
        void recvLoop(recv_args *arg);

        /**
         * 发送数据的线程，
        */
        void sendLoop(send_args *arg);

        /**
         * 等待发送和接受结束， 作用类似于 ros::Spin();
        */
        void join()
        {
            recvThread.join();
            sendThread.join();
        }

    private:
        // 线程们
        std::thread recvThread;
        std::thread sendThread;
        // std::thread sendThread50;

    public:
        
        std::mutex recv_data_lock;
        unsigned int opened=0;
        CMSG recv[1000];// tj 接收到的数据,50个 我也不知道为什么，这个调大了延时会短比较好
        //tj int reclen = 0;


    private:

        unsigned int DeviceChannel = 0;
        NTCAN_HANDLE dev_handler_;//tj 相当esd can卡里给开起来的channel的一个编号

    };

    /*---------------------编码部分，工控机算法得到的物理数据通过CAN发给底层控制器--------------*/
    std::vector<int> generate_motorola_bit_order(int msb, int lsb, int size);

    template <typename T>
    void encodeMsg(T data, int lsb, int msb, int size,
        double offset, double factor, uint8_t* output) {
        data = static_cast<T>((data - offset) / factor);
        uint64_t temp = static_cast<uint64_t>(data) & ((1ULL << size) - 1);

        std::vector<int> bit_order = generate_motorola_bit_order(msb, lsb, size);

        for (int i = 0; i < size; ++i) {
            int bit_index = bit_order[i];
            int byte = bit_index / 8;
            int bit = bit_index % 8;

            output[byte] &= ~(1 << bit);
            if (temp & (1ULL << (size - 1 - i))) {
                output[byte] |= (1 << bit);
            }
        }
    }

    template <typename T>
    T decodeMsg(uint8_t* msg, int lsb, int msb, int size,
        double offset, double factor) {
        uint64_t temp = 0;
        std::vector<int> bit_order = generate_motorola_bit_order(msb, lsb, size);

        for (int i = 0; i < size; ++i) {
            int bit_index = bit_order[i];
            int byte = bit_index / 8;
            int bit = bit_index % 8;
            if (msg[byte] & (1 << bit)) {
                temp |= (1ULL << (size - 1 - i));
            }
        }
        return static_cast<T>(temp * factor + offset);
    }
}

#endif