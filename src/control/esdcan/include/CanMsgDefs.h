#ifndef CANMSGDEFS_H
#define CANMSGDEFS_H

#include "CanHandler.h"
#include <vector>
#include <map>
#include <iostream>

namespace CanProcess
{   

    // 单一的一条消息， 对应excel表中的一行
    template <typename T>
    struct SingleMsg
    {
        std::string name = "";
        uint8_t lsb = 0;
        uint8_t msb = 0;
        uint8_t size = 0;
        double offset = 0;
        double factor = 1;
        T data = 0;
        SingleMsg(){};
        // 构造函数
        SingleMsg(std::string name, uint8_t lsb, uint8_t msb, uint8_t size, double offset = 0, double factor = 1)
            : name(name), lsb(lsb), msb(msb), size(size), offset(offset), factor(factor)
        {
        }
        // 拷贝构造函数
        SingleMsg(const SingleMsg &copy) : name(copy.name), lsb(copy.lsb), msb(copy.msb), size(copy.size), offset(copy.offset), factor(copy.factor), data(copy.data)
        {
        }
    };

    /* 预先将这个车上的can消息分类为
        double 小数值
        int 整数状态值
        bool 2进制状态值
     */
    typedef SingleMsg<double> SingleMsg_double;
    typedef SingleMsg<int> SingleMsg_int;
    typedef SingleMsg<bool> SingleMsg_bool;

    // 一条Can消息， 对应 一帧CMSG， 只有1个id (64个bit)
    class CAN_Message
    {
    public:
        unsigned int id = 0;
        // map容器， 类似于python中的字典， key为消息名字， 直接用excel上的名字就可以定位
        std::map<std::string, SingleMsg<double>> all_double_data;
        std::map<std::string, SingleMsg<int>> all_int_data;
        std::map<std::string, SingleMsg<bool>> all_bool_data;

        CAN_Message() {}
        CAN_Message(unsigned int id) : id(id) {}

        /** 
         *  将这条 Can 消息转换为 CMSG,
         * @param dataframe 指向目标消息的指针， 注意必须在调用前分配内存
         */
        void toDataFrame(CMSG *dataframe);
    };
    
    //  储存着所有的 发送类消息(指令) 的 map容器， key为 消息的帧ID
    extern std::map<unsigned int, CAN_Message> all_can_send;
    /*
     初始化所有的 发送类消息， 也就是抄一遍excel表
     保存了每个单一消息的 lsb， msb， size， offset， factor 等重要信息
    */
    void initSendMessages();

    //  储存着所有的 接受类消息(反馈) 的 map容器， key为 消息的帧ID
    extern std::map<unsigned int, CAN_Message> all_can_recv;
    // 初始化所有的 接受类消息， 也就是抄一遍excel表
    void initRecvMessage();

    // 将 CAN 消息 解析为数据， 存在 all_can_recv 中
    void canToHuman(CMSG *df, int size);
}

#endif