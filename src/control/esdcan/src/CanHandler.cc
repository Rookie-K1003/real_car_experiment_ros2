#include "CanHandler.h"
//tj #include "ICANCmd.h"
#include "CanMsgDefs.h"
#include <iostream>

//using namespace std;
namespace CanProcess
{
    struct recv_args;
    struct send_args;
    // 接受线程的参数
    recv_args recvArg;

    // 发送线程的参数， sendArg50 是间隔为50ms的线程， sendArg对应间隔为20ms的线程
    send_args sendArg;
    //tj send_args sendArg50;

    // 用于测试的线程锁
    std::mutex testLock;
    int test = 0;
    

    // 用于控制线程阻塞时间，从而达到定频率运行的功能，
    auto now() { return std::chrono::steady_clock::now(); }
    // a为毫秒
    auto awake_time(int a) { return now() + std::chrono::milliseconds(a); }



    /**
     * Create a CAN processor
    */
    CanHandler::CanHandler()
    {
        recvThread = std::thread(&CanHandler::recvLoop, this, &recvArg);
        sleep(1);
        sendThread = std::thread(&CanHandler::sendLoop, this, &sendArg);
    }

    /**
     * Create a CAN processor
     * tj @param DevType 设备类型
     * tj @param Idex USB索引， 从0开始
     * @param Channel 对应CAN卡上的CAN通道， 1代表 CAN0， 2代表 CAN1， 3代表CAN0和CAN1
     * tj @param config CAN卡的设定， 具体参考CanHandler.h 
    */
    // tj CanHandler::CanHandler(int DevType, int Idex, int Channel, CAN_InitConfig config) : DeviceType(DevType), DeviceIdx(Idex), DeviceChannel(Channel), Can_Config(config)
    CanHandler::CanHandler(int Channel) : DeviceChannel(Channel)
    {
        /*tj*/

        // 打开 Can设备
        opened = OpenDevice(Channel);
        /*tj*/
        recvArg.channel=Channel;
        recvThread = std::thread(&CanHandler::recvLoop, this, &recvArg);
        LOG(INFO)<<"recv thread created!!";
        sleep(1);
        
        sendArg.channel=Channel;
        sendThread = std::thread(&CanHandler::sendLoop, this, &sendArg);
        LOG(INFO)<<"send thread created!!";
        
    }

    /**
     * 接受数据的线程
     * @arg 接受数据的参数
    */
    void CanHandler::recvLoop(recv_args *arg)
    {
        while (arg->run)
        {

            int32_t frame_num=1000;//接受帧数 我也不知道为什么，这个调大了延时会短比较好
            const int ret = canTake(dev_handler_, recv, &frame_num);
            
            // rx timeout not log
            if (ret == NTCAN_RX_TIMEOUT) {
                std::cout << "read timeout\n";
            }
            else if (ret != NTCAN_SUCCESS) {
                std::cout << "read error\n";
            }
            else 
            {
                // std::cout << "read success\n";
                std::unique_lock<std::mutex> lock(recv_data_lock);
                //std::cout<<"frame_num"<<frame_num<<std::endl;
                /*
                for(int i=0;i<frame_num;++i)
                {
                    std::cout<<std::hex<<recv[i].id<<std::endl;
                }
                */
                
                
    
                canToHuman(recv, frame_num);     
            }
            
            usleep(1000*100);

        }
        return;
    }

    /**
     * 发送数据的线程，
    */

    void CanHandler::sendLoop(send_args *arg)
    {
        
        while (arg->run)
        {
            
            if (!dev_handler_)
                continue;
            
            if (!arg->msg_arrived)//等到arg->data里面有东西才能发送
                continue;
            // LOG(INFO) << "Message Arrived! \n Send interval: " << arg->interval << "\n";
            // LOG(INFO) << "Message Count: " << arg->num_frames;
            int msgSend = 0;
            int send_num=arg->num_frames;

            if(1)
            {
                std::unique_lock<std::mutex> lock(arg->datalock);

                //std::cout<<"sendid"<<std::hex<<arg->data[0].id<<std::hex<<arg->data[1].id<<std::hex<<arg->data[2].id<<arg->data[3].id<<std::endl;
                msgSend = canSend(dev_handler_, arg->data, &send_num);
            }

            if (msgSend != NTCAN_SUCCESS) {
                std::cout << "send error\n";std::cout<<"msgSend"<<msgSend;
            }
            
            else {
                //std::cout << "send success\n";
            }

            

            std::this_thread::sleep_until(awake_time(arg->interval));
        }
        return;
    }

}