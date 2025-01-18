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
    /**
     * Encode data to can message
     * Intel 字节序编码 CAN 消息，萌健号上的 VCU 通信协议中用到
     * @param data 可以选择不同类型的数据， 例如 bool int double 等等
     * @param lsb @param msb @param size @param offset @param factor 与 excel表中的消息属性对应
    */
    // template <typename T>  // 模板函数必须定义在头文件中， 否则无法链接
    // void encodeMsg(T data, int lsb, int msb, int size, double offset, double factor, uint8_t *output)
    // {
    //     data = (data - offset) / factor;
    //     // 确定起始的字节位置，也就是layout表上，消息开始的行数（不是bit位置）
    //     uint8_t *startByte = output + lsb / 8;
    //     int length = msb / 8 - lsb / 8 + 1;

    //     uint64_t temp(data);
    //     // 截断该数据， 防止数据溢出
    //     uint64_t i = (1 << size) - 1;
    //     temp &= i;

    //     // 移动位置到合适的位置（按照lsb移动，确保数据在合适的位置）
    //     temp <<= (lsb % 8);

    //     for (int i = 0; i < length; i++)
    //     {
    //         int idx = length - 1 - i;
    //         startByte[idx] |= ((temp >> (8 * (idx))) & (255));
    //     }
    // }

    /**
     * Encode data to CAN message in Motorola (Big Endian) byte order
     * 20241205 用于 Motorola (Big Endian) 字节序编码 CAN 消息，美团车上的VCU通信协议中用到
     * @param data 可以选择不同类型的数据，例如 bool int double 等等
     * @param lsb @param msb @param size @param offset @param factor 与 Excel 表中的消息属性对应
     */
    // template <typename T>
    // void encodeMsg(T data, int lsb, int msb, int size, double offset, double factor, uint8_t *output)
    // {
    //     // 计算数据的原始值
    //     data = (data - offset) / factor;

    //     // 确定起始的字节位置
    //     uint8_t *startByte = output + lsb / 8;
    //     int length = msb / 8 - lsb / 8 + 1;

    //     // 转换数据为无符号长整型
    //     uint64_t temp = static_cast<uint64_t>(data);

    //     // 截断该数据，防止溢出
    //     uint64_t mask = (1ULL << size) - 1;  // 使用 1ULL 确保是无符号长整型
    //     temp &= mask;

    //     // 移动位置到合适的位置（按照 lsb 移动，确保数据在合适的位置）
    //     temp <<= (lsb % 8);

    //     // 按照大端字节序写入数据
    //     for (int i = 0; i < length; i++)
    //     {
    //         int idx = i;  // 大端字节序，低位字节在后面
    //         startByte[idx] |= ((temp >> (8 * (length - 1 - idx))) & 0xFF);
    //     }
    // } // 此版本经过尝试，数据发出后底层收的不对 20241206 @kq 存疑，需要用vs调试查一下


    /**
     * 20241206 Encode data to CAN message in Motorola (Big Endian) byte order V3版本
     * 为避免msb可能比lsb小的情况（跨字节时），所以计算逻辑中只用lsb
     * @param data 实际数据，可以是不同类型，例如 int、float 等
     * @param msg CAN 消息数据缓冲区
     * @param lsb 该信号的起始位
     * @param msb 该信号的结束位
     * @param size 信号的位大小
     * @param offset 偏移量
     * @param factor 缩放因子
     */
    template <typename T>
    void encodeMsg(T data, int lsb, int msb, int size, double offset, double factor, uint8_t* output)
    {
        // Apply offset and factor to data
        data = static_cast<T>((data - offset) / factor);

        // Truncate the data to the specified size
        uint64_t temp = static_cast<uint64_t>(data) & ((1ULL << size) - 1);

        // Determine the starting byte position
        int startByte = lsb / 8;
        int startBit = lsb % 8;

        // Clear the output buffer for the given range
        for (int i = startByte; i <= (msb / 8); ++i) {
            output[i] = 0;
        }

        // Fill in the bits
        for (int i = 0; i < size; ++i) {
            if (temp & (1ULL << i)) {
                output[startByte - (startBit + i) / 8] |= (1 << ((startBit + i) % 8));
            }
        }
        
    } 

    
    /*---------------------解码部分，VCU通过CAN发来的转为实际物理数据值------------------------*/
    /**
     * Decode can message to data, 上面全部反向操作就行了
     * 用于解码Intel字节序的 CAN 消息，萌健号上的 VCU 通信协议中用到
     * @param data 可以选择不同类型的数据， 例如 bool int double 等等
     * @param lsb @param msb @param size @param offset @param factor 与 excel表中的消息属性对应
    */
    // template <typename T>
    // T decodeMsg(uint8_t *msg, int lsb, int msb, int size, double offset, double factor)
    // {
    //     int64_t temp = 0;
    //     uint8_t *startByte = msg + lsb / 8;
    //     int length = msb / 8 - lsb / 8 + 1;

    //     for (int i = 0; i < length; i++)
    //     {
    //         int idx = length - 1 - i;
    //         temp += (startByte[idx] << (8 * idx));
    //     }
    //     temp >>= (lsb % 8);
    //     int64_t i = ((static_cast<int64_t>(1) << size) - static_cast<int64_t>(1));
    //     temp &= i;
    //     return T((temp * factor + offset));
    // }

    /**
     * Decode CAN message to data in Motorola (Big Endian) byte order
     * 20241205 用于 Motorola (Big Endian) 字节序解码 CAN 消息，美团车上的VCU通信协议中用到
     * @param msg CAN 消息数据
     * @param lsb 该信号的起始位
     * @param msb 该信号的结束位
     * @param size 信号的位大小
     * @param offset 偏移量
     * @param factor 缩放因子
     * @return 解码后的数据
     */
    template <typename T>
    T decodeMsg(uint8_t *msg, int lsb, int msb, int size, double offset, double factor)
    {
        // 以实际转向角度为例
        int64_t temp = 0;
        uint8_t *startByte = msg + lsb / 8;
        int length = msb / 8 - lsb / 8 + 1;

        // 按照大端字节序读取数据
        for (int i = 0; i < length; i++)
        {
            // 直接使用 i，表示从高字节到低字节
            temp += (startByte[i] << (8 * (length - 1 - i)));
        }

        // 右移以调整到正确的位位置
        temp >>= (lsb % 8);

        // 创建一个掩码用于截断多余的位
        int64_t mask = (static_cast<int64_t>(1) << size) - 1;
        temp &= mask;

        // 应用因子和偏移量，返回解码后的数据
        return T((temp * factor + offset));
    }
}

#endif