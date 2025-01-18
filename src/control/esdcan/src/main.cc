//#define CAN_Device USBCAN_E_1CH
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include "CanMsgDefs.h"
#include <iostream>
#include "CanCommonFcn.h"
#include "CanHandler.h"
//tj #include "ICANCmd.h"
//#include "ntcan.h"
#include "math.h"
#include "control_msgs/msg/control_req.hpp"
#include "location_msgs/msg/rtk.hpp"
#include "std_msgs/msg/float64.hpp"


//#include "std_msgs/Int8.h"
#include <thread>
#include <mutex>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
using namespace CanProcess;

class esdcan : public rclcpp::Node
{
    public:
        explicit esdcan(const rclcpp::NodeOptions & node_options): Node("esdcan", node_options)
        {

            subTrackingControlCmd = this->create_subscription<control_msgs::msg::ControlReq>("/tracking_control_req", 1, std::bind(&esdcan::CallbackTrackingControlCmd, this, std::placeholders::_1));
            subRtkData = this->create_subscription<location_msgs::msg::RTK>("/rtk_data", 1, std::bind(&esdcan::CallbackRtkData, this, std::placeholders::_1));

            pubSpeedAndAngle = create_publisher<geometry_msgs::msg::TwistStamped>("/current_vehicle_can_info", 1);
            pubCurrentAcceleration = create_publisher<std_msgs::msg::Float64>("/curAccel", 1);
            pubTargetAcceleration = create_publisher<std_msgs::msg::Float64>("/tarAccel", 1);
            pubTargetSpeed = create_publisher<std_msgs::msg::Float64>("/tarSpeed", 1);
            pubTargetAngle = create_publisher<std_msgs::msg::Float64>("/tarAngle", 1);
            

        }
        ~esdcan(){}
        void main_process()
        {
            initRecvMessage();
            initSendMessages();
            CanHandler CanIO(channel);
            //tj unsigned int opened;
            // 打开 Can设备
            //tj opened = CanIO.OpenDevice();

            // 获取这几个参数的地址， 用于后续控制发送什么消息
            send_args *send = &sendArg;
            // send_args *send50 = &sendArg50;
            recv_args *recv = &recvArg;
            recv->channel = channel; // 0 是channel 0,，默认为0
            send->channel = channel;

            // 这里可以设置发送进程的间隔， 单位是毫秒上升沿
            send->interval = 50;

            
            // 设置上升沿 进入自动加速模型
            if (CanIO.opened != 0)
            {

                send->num_frames = 4;
                send->data = new CMSG[send->num_frames];//之前是空指针，所以赋值。
                for(int i=0;i<send->num_frames;++i)send->data[i].len=8;//这个也要写，是can卡必须的参数

                {
                    std::unique_lock<std::mutex> lock(CanIO.recv_data_lock);
                    g_CurrentAngle = getTurn(g_car);
                }
                //LOG(INFO) << "Initial Angle: " << angle << "\n\n";
                setTurn(g_CurrentAngle, g_car);
                // enterTurn();
                // enterDrive();
                // enterGear();
                send_all_safe(send);
                
            }

            rclcpp::WallRate loop_rate(20); // 20Hz
            setTurn(0, g_car);
            rclcpp::Node::SharedPtr node_(this);

        #ifdef DEBUG

            std::thread keyboard(getKeyBoardInput);
            keyboard.detach();

        #endif
            while (rclcpp::ok())
            {
                
                if (CanIO.opened == 0)
                {
                    std::cout<<"THE CAN IS NOT OPEN!!!";
                    continue;
                }
                rclcpp::spin_some(node_);
                std::unique_lock<std::mutex> lock(CanIO.recv_data_lock);

                g_Direction = g_TargetGear == 2 ? -1 : 1;//0N 1D 2R

                g_TargetSpeed = std::abs(g_TargetSpeed);

                // g_CurrentAcceleration = g_Direction * getAcc(); //倒车刹车时加速度 应为负
                g_CurrentAngle = getTurn(g_car);

                

                /* --------------------------- DEBUG 模式下 根据键盘调节车速 --------------------------- */

        #ifdef DEBUG

                if (g_KeyBoardInput == 'w')
                {
                    g_TargetSpeed += 0.2;
                    std::cerr << "Current speed up: " << g_TargetSpeed << std::endl;
                    {
                        std::lock_guard<std::mutex> lock(g_KeyBoardMutex);
                        g_KeyBoardInput = 0;
                    }
                }
                else if (g_KeyBoardInput == 's')
                {
                    g_TargetSpeed -= 0.2;
                    if (g_TargetSpeed < 0)
                    {
                        g_TargetSpeed = 0;
                    }
                    std::cerr << "Current speed down: " << g_TargetSpeed << std::endl;
                    {
                        std::lock_guard<std::mutex> lock(g_KeyBoardMutex);
                        g_KeyBoardInput = 0;
                    }
                }
                // std::cerr << "Current target speed: " << g_TargetSpeed << std::endl;

        #endif
                // 根据期望速度分段pid控制 输出加速度
                // g_TargetAcceleration = setTargetAccel(g_TargetSpeed, g_CurrentSpeed, g_LastError, g_AccumulatedError, g_PidVoltage);
                // std::cerr << "DIRECTION:" << g_Direction << std::endl;
        #ifdef DEBUG
                if (g_Direction == 1)
                {
                    g_TargetGear = 3;
                }
                else if (g_Direction == -1 && g_CurrentSpeed < 1)
                {
                    g_TargetGear = 1;
                }
                setTurn(0);

        #endif

        #ifdef FILTER
                    lastTargetAccel = g_TargetAcceleration;
                    currTargetAccel = filterTimeConstant * g_TargetAcceleration +
                                    (1 - filterTimeConstant) * lastTargetAccel;
                    g_TargetAcceleration = currTargetAccel;
                    lastTargetAccel = currTargetAccel;
        #endif

                setTurn(g_TargetAngle, g_car); // 设置方向盘转角 deg

                // 如果期望速度为 0，则发送固定加速度保证其刹停；
                if (std::abs(g_TargetSpeed) < 0.1)
                {
                    //try printf("Decrease to stop......\n");
                    g_TargetAcceleration = -2.0;

                }
                // 怠速外 加速度做限制 不然这个限制会对怠速时加减速模式切换有影响
                else if (std::abs(g_TargetSpeed) > 1.1)
                {
                    g_TargetAcceleration = std::min(g_CurrentAcceleration + 1.2, g_TargetAcceleration);
                    g_TargetAcceleration = std::max(g_CurrentAcceleration - 1.2, g_TargetAcceleration);
                }

               //打印数据
                std::cout<<" the target angle , speed , acc is : "
                        << g_TargetAngle << "   "
                        << g_TargetSpeed << "   " 
                        << g_TargetAcceleration << std::endl;

                std::cout<<" the current angle , speed , acc is : "
                        << g_CurrentAngle << "   "
                        << g_CurrentSpeed << "   " 
                        << g_CurrentAcceleration << std::endl;

                send->msg_arrived = true;//正式开始发送


                // 向can发送报文
                send_all_safe(send);

                loop_rate.sleep();
            }
            int numtj=0;
            while(numtj<10)
            {
            setTurn(0, g_car);

            send_all_safe(send);
            numtj++;
            }
            sleep(1);
            numtj=0;
            while(numtj<10)
            {
            // setAcc(0,g_CurrentSpeed);
            send_all_safe(send);
            numtj++;
            }


            sleep(1);

            exitAll(g_car);
            
            send_all_safe(send);sleep(2);

            delete[] send->data;

            //sleep(3);
            send->run = false;
            recv->run = false;
            //todo
            //检测各模式的自动驾驶是否退出
            CanIO.join();
        }

    private:

        double g_LastError = 0.0; //定义上一个偏差值
        double g_AccumulatedError = 0.0;
        double g_PidVoltage = 0.0; //定义控制执行器变量

        int g_Direction = 1; //1 - 前进 -1 - 倒退
        int g_BreakHandBrakeCount = 0;

        // #define DEBUG
        // char g_KeyBoardInput = '\0';
        int g_KeyBoardInput = 0;
        int g_TurningSystemStatus;

        int g_TargetGear = 1;
        double g_TargetAngle = 0.0;
        double g_TargetSpeed = 0;
        double g_TargetAcceleration = 0.0;

        int g_CurrentGear;
        double g_CurrentAngle = 0.0;
        double g_CurrentSpeed = 0.0;
        double g_CurrentAcceleration = 0.0;

        int g_LongitudeDrivingStatus = 0;
        int g_HandBrakeStatus = 1;

        int g_CurrentBehaviorState = 0;

        int channel = 2; // channel选择2
        int g_car = 1; // 1:美团车 0：萌建号 
   
        std::mutex g_KeyBoardMutex;
        std::mutex g_CurrentBehaviorMutex;
        std::mutex g_SpeedMutex;
        rclcpp::Subscription<control_msgs::msg::ControlReq>::SharedPtr subTrackingControlCmd;
        rclcpp::Subscription<location_msgs::msg::RTK>::SharedPtr subRtkData;
    
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubSpeedAndAngle;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubCurrentAcceleration;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubTargetAcceleration;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubTargetSpeed;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubTargetAngle;



        void getKeyBoardInput()
        {
            rclcpp::WallRate loopRate(50);
            while (1)
            {
                // std::cerr << "waiting for input" << std::endl;
                g_KeyBoardInput = getchar();
                int enter = getchar(); // =='\n'
                // std::cerr << "g_KeyBoardInput:" << g_KeyBoardInput << std::endl;
                loopRate.sleep();
            }
        }

        void CallbackTrackingControlCmd(const control_msgs::msg::ControlReq::ConstPtr controlCmdMsg)
        {
            int currentBehaviorState;
            {
                std::lock_guard<std::mutex> lock(g_CurrentBehaviorMutex);
                currentBehaviorState = g_CurrentBehaviorState;
            }
            if (currentBehaviorState == 16) //! parking_state is 16 !!!!!
                return;

            // std::cout << "LikeCAN receives control msgs" << "\n";
            g_TargetSpeed = controlCmdMsg->vel_req;
            int temp=controlCmdMsg->gear_req;
            if (temp==1)g_TargetGear = 2;//同济车档位
            if (temp==2)g_TargetGear = 0;
            if (temp==3)g_TargetGear = 1;
            
            g_TargetAngle = controlCmdMsg->angle_req;
            // std::cout << "LikeCAN receives speed and gear and angle:  " << g_TargetSpeed << "   " << g_TargetGear << "     " << g_TargetAngle <<"\n";

        }
       
        void CallbackRtkData(const location_msgs::msg::RTK::ConstPtr Msg)
        {
            // std::lock_guard<std::mutex> lock(g_SpeedMutex);
            g_CurrentSpeed = Msg->velocity;
            g_CurrentAcceleration = Msg->accel_raw_y - 0.15;
        }

};


int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    //rclcpp::WallRate loop_rate(20); // 20Hz
    auto esdcaner = std::make_shared<esdcan>(rclcpp::NodeOptions{});
    esdcaner->main_process();
}
