#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msgs/Imu.hpp>
// #include <geometry_msgs/msgs/PoseStamped.hpp>
// #include <geometry_msgs/msgs/TransformStamped.hpp>
//#include "ICANCmd.h"
#include <ntcan.h>
#include "location_msgs/msg/rtk.hpp"
using namespace std;
#define NEWFLAG -100000
NTCAN_HANDLE dev_handler_;//tj 相当esd can卡里给开起来的channel的一个编号

//DWORD dwDeviceHandle;
//CAN_InitConfig config;

class can_decode_class : public rclcpp::Node
{
    public:
        explicit can_decode_class(const rclcpp::NodeOptions & node_options): Node("can_decode", node_options)
        {
            rtk_pub = create_publisher<location_msgs::msg::RTK>("rtk_data", 1000);
        };
        ~can_decode_class(){};
        void main_process()
        {
                int channel=3;//channel

                uint32_t mode = 0;//应该默认就是0；拓展帧是0x10；

                int32_t ret = canOpen(channel, mode, NTCAN_MAX_TX_QUEUESIZE,
                NTCAN_MAX_RX_QUEUESIZE, 5, 5, &dev_handler_);
                if (ret != NTCAN_SUCCESS) {
                    std::cout << "open device error code:\n";
                    std::cout << "ret=\n"<<ret;} 
                else {
                std::cout << "open device sussess:\n";}

                // 1. set receive message_id filter, ie white list 大概是有个CANid的滤波器，要把要的发过去
                int32_t id_count = 0x800;
                ret = canIdRegionAdd(dev_handler_, 0, &id_count);//相当于从0到0+0x800都会被通过

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
                //sleep(1);

                /*tj

                if ((dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B, 1, 0)) == 1)
                {
                    ROS_INFO_STREAM(" >>open device success!");
                }
                // else if ((dwDeviceHandle = CAN_DeviceOpen(LCUSB_132B, 1, 0)) == 1)
                // {
                //     ROS_INFO_STREAM(" >>open device success!");
                // }
                // else if ((dwDeviceHandle = CAN_DeviceOpen(LCUSB_132B, 2, 0)) == 1)
                // {
                //     ROS_INFO_STREAM(" >>open device success!");
                // }
                else
                {
                    ROS_ERROR_STREAM(" >>open device error!");
                    return 0;
                    exit(1);
                }

                CAN_InitConfig config;
                config.dwAccCode = 0;
                config.dwAccMask = 0xffffffff;
                config.nFilter = 0;     // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
                config.bMode = 0;       // 工作模式(0表示正常模式,1表示只听模式)
                config.nBtrType = 1;    // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
                config.dwBtr[0] = 0x00; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
                config.dwBtr[1] = 0x1c; // BTR1
                config.dwBtr[2] = 0;
                config.dwBtr[3] = 0;

                if (CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK)
                {
                    ROS_ERROR_STREAM(" >>Init CAN0 error!");

                    return 0;
                }

                if (CAN_ChannelStart(dwDeviceHandle, 1, &config) != CAN_RESULT_OK)
                {
                    ROS_ERROR_STREAM(" >>Init CAN1 error!");

                    return 0;
                }
                */

                //tj int reclen = 0;
                CMSG recv[1000]; //buffer 我也不知道为什么，这个调大了延时会短比较好
                //tj int CANInd = 0;          //CAN1=0, CAN2=1


                nav_msgs::msg::Path pathGT;
                pathGT.header.frame_id = "/imu_init";

                location_msgs::msg::RTK rtkdata;
                rtkdata_reset(rtkdata);

                bool init = 0;
                Eigen::Matrix3d R_ENU2IMU;
                Eigen::Matrix3d R_ENU;
                Eigen::Matrix3d R_IMU;
                Eigen::Vector3d t_ECEF2ENU(0, 0, 0);
                Eigen::Vector3d t_ECEF(0, 0, 0);
                Eigen::Vector3d t_ENU(0, 0, 0);
                Eigen::Vector3d t(0, 0, 0);
                Eigen::Quaternion<double> q(1, 0, 0, 0);
                const double e = 0.0818191908425;
                const double R = 6378137;
                const double torad = M_PI / 180;
                const double g = 9.7964;
                double Re = 0;        
                
                //int32_t ret2=-1;//是否接收成功
                rclcpp::Rate r(100);

                while (rclcpp::ok())
                {
                    // printf("begin collect can data");
                    int32_t frame_num=100;//大概接受30帧,而且必须放在这里声明，因为这个值输入cantake函数作为计划接收的数量，同时被输出实际接收的数量，必须每个循环都重新定义 我也不知道为什么，这个调大了延时会短比较好

                    // std::cout<<"frame_num1:  "<<frame_num<<std::endl;
                    const int ret2 = canTake(dev_handler_, recv, &frame_num);
                    // std::cout<<"frame_num2:  "<<frame_num<<std::endl;
                    // std::cout<<"ret2:  "<<ret2<<std::endl;


                    if (ret2 ==NTCAN_SUCCESS&&frame_num!=0) //调用接收函数,得到数据
                    {
                        printf("begin decode");
                        for (int i = 0; i<frame_num;++i)
                        {
                            //std::cout<<"id"<<std::hex<<recv[i].id;
                            
                            if (recv[i].id == 0x320)
                            {
                                rtkdata.gpstime = CAN_decode(recv[i], 40, 32, 1e-3, 0, 0);
                            }
            
                            if (recv[i].id == 0x324)
                            {
                                rtkdata.latitude = CAN_decode(recv[i], 24, 32, 1e-7, 0, 0);
                                rtkdata.longitude = CAN_decode(recv[i], 56, 32, 1e-7, 0, 0);
                                //printf("longitude : %f , latitude : %f \n", rtkdata.longitude, rtkdata.latitude);
                            }

                            if (recv[i].id == 0x321)
                            {
                                rtkdata.angrate_raw_x = CAN_decode(recv[i], 20, 20, 1e-2, 0, 1) * torad;
                                rtkdata.angrate_raw_y = CAN_decode(recv[i], 32, 20, 1e-2, 0, 1) * torad;
                                rtkdata.angrate_raw_z = CAN_decode(recv[i], 60, 20, 1e-2, 0, 1) * torad;
                            }

                            if (recv[i].id == 0x322)
                            {
                                rtkdata.accel_raw_x = CAN_decode(recv[i], 20, 20, 1e-4, 0, 1) * g;
                                rtkdata.accel_raw_y = CAN_decode(recv[i], 32, 20, 1e-4, 0, 1) * g;
                                rtkdata.accel_raw_z = CAN_decode(recv[i], 60, 20, 1e-4, 0, 1) * g;
                            }

                            if (recv[i].id == 0x325)
                            {
                                rtkdata.height = CAN_decode(recv[i], 24, 32, 1e-3, 0, 0);
                                //printf("height : %f \n", rtkdata.height);
                            }

                            if (recv[i].id == 0x32A)
                            {
                                rtkdata.heading = CAN_decode(recv[i], 8, 16, 1e-2, 0, 0);
                                rtkdata.pitch = CAN_decode(recv[i], 24, 16, 1e-2, 0, 1);
                                rtkdata.roll = CAN_decode(recv[i], 40, 16, 1e-2, 0, 1);
                                // printf("heading : %f , pitch : %f , roll : %f \n", rtkdata.heading, rtkdata.pitch, rtkdata.roll);
                            }

                            if (recv[i].id == 0x323)
                            {
                                rtkdata.status = CAN_decode(recv[i], 16, 8, 1, 0, 0);
                            }
                            if (recv[i].id == 0x327)
                            {
                                rtkdata.velocity = CAN_decode(recv[i], 56, 16, 1e-2, 0, 0);
                            }
                        }
                        //ROS_INFO("finished decode");
                    }

                    if (rtkdata_isNew(rtkdata))
                    {
                        std::cout<<"begin publish";
                        rtkdata.stamp = rclcpp::Clock ().now ();
                        // if (!init)
                        // {
                        //     R_ENU2IMU = Eigen::AngleAxisd(rtkdata.heading * torad, Eigen::Vector3d::UnitZ()) *
                        //                 Eigen::AngleAxisd(rtkdata.roll * torad, Eigen::Vector3d::UnitY()) *
                        //                 Eigen::AngleAxisd(rtkdata.pitch * torad, Eigen::Vector3d::UnitX());
                        //     Re = R / sqrt(1 - e * e * sin(rtkdata.latitude * torad) * cos(rtkdata.longitude * torad));
                        //     t_ECEF2ENU[0] = (Re + rtkdata.height) * cos(rtkdata.latitude * torad) * cos(rtkdata.longitude * torad);
                        //     t_ECEF2ENU[1] = (Re + rtkdata.height) * cos(rtkdata.latitude * torad) * sin(rtkdata.longitude * torad);
                        //     t_ECEF2ENU[2] = (Re * (1 - e * e) + rtkdata.height) * sin(rtkdata.latitude * torad);
                        //     init = !init;
                        // }
                        // R_ENU = Eigen::AngleAxisd(rtkdata.heading * torad, Eigen::Vector3d::UnitZ()) *
                        //         Eigen::AngleAxisd(rtkdata.roll * torad, Eigen::Vector3d::UnitY()) *
                        //         Eigen::AngleAxisd(rtkdata.pitch * torad, Eigen::Vector3d::UnitX());
                        // R_IMU = R_ENU * R_ENU2IMU.inverse();
                        // q = R_IMU.inverse();

                        // t_ECEF[0] = (Re + rtkdata.height) * cos(rtkdata.latitude * torad) * cos(rtkdata.longitude * torad) - t_ECEF2ENU[0];
                        // t_ECEF[1] = (Re + rtkdata.height) * cos(rtkdata.latitude * torad) * sin(rtkdata.longitude * torad) - t_ECEF2ENU[1];
                        // t_ECEF[2] = (Re * (1 - e * e) + rtkdata.height) * sin(rtkdata.latitude * torad) - t_ECEF2ENU[2];

                        // t_ENU[0] = -sin(rtkdata.longitude * torad) * t_ECEF[0] + cos(rtkdata.longitude * torad) * t_ECEF[1];
                        // t_ENU[1] = -sin(rtkdata.latitude * torad) * cos(rtkdata.longitude * torad) * t_ECEF[0] - sin(rtkdata.latitude * torad) * sin(rtkdata.longitude * torad) * t_ECEF[1] + cos(rtkdata.latitude * torad) * t_ECEF[2];
                        // t_ENU[2] = cos(rtkdata.latitude * torad) * cos(rtkdata.longitude * torad) * t_ECEF[0] + cos(rtkdata.latitude * torad) * sin(rtkdata.longitude * torad) * t_ECEF[1] + sin(rtkdata.latitude * torad) * t_ECEF[2];

                        // t = R_ENU2IMU * t_ENU;
                        // nav_msgs::Odometry odomGT;
                        // odomGT.header.frame_id = "/imu_init";
                        // odomGT.child_frame_id = "/rtk_truth";

                        // odomGT.header.stamp = rtkdata.stamp;
                        // odomGT.pose.pose.orientation.x = q.x();
                        // odomGT.pose.pose.orientation.y = q.y();
                        // odomGT.pose.pose.orientation.z = q.z();
                        // odomGT.pose.pose.orientation.w = q.w();
                        // odomGT.pose.pose.position.x = t(0);
                        // odomGT.pose.pose.position.y = t(1);
                        // odomGT.pose.pose.position.z = t(2);

                        // geometry_msgs::TransformStamped trans;
                        // trans.header = odomGT.header;
                        // trans.transform.translation.x = t(0);
                        // trans.transform.translation.y = t(1);
                        // trans.transform.translation.z = t(2);
                        // trans.transform.rotation.x = q.x();
                        // trans.transform.rotation.y = q.y();
                        // trans.transform.rotation.z = q.z();
                        // trans.transform.rotation.w = q.w();
                        // pubPose.publish(trans);

                        // geometry_msgs::PoseStamped poseGT;
                        // poseGT.header = odomGT.header;
                        // poseGT.pose = odomGT.pose.pose;
                        // pathGT.header.stamp = odomGT.header.stamp;
                        // pathGT.poses.push_back(poseGT);
                        // pubPathGT.publish(pathGT);

                        // odomGT.pose.pose.position.x = 0;
                        // odomGT.pose.pose.position.y = 0;
                        // odomGT.pose.pose.position.z = 0;
                        // pubOdomGT.publish(odomGT);

                        rtk_pub->publish(rtkdata);
                        rtkdata_reset(rtkdata);
                        std::cout<<"finished publish";
                        
                    }
                    r.sleep();

                    //ROS_INFO("enter next loop");
                    }

                    int32_t ret3 = canClose(dev_handler_);
                    if (ret3 != NTCAN_SUCCESS) {
                    std::cout << "close error code:\n";} 
                    else {
                    std::cout << "close esd can ok. port\n";}

        }
    private:
        rclcpp::Publisher<location_msgs::msg::RTK>::SharedPtr rtk_pub;
        void rtkdata_reset(location_msgs::msg::RTK &rtkdata)
        {
            rtkdata.gpstime = NEWFLAG;
            // rtkdata.longitude = NEWFLAG;
            // rtkdata.latitude = NEWFLAG;
            // rtkdata.height = NEWFLAG;
            // rtkdata.heading = NEWFLAG;
            // rtkdata.roll = NEWFLAG;
            // rtkdata.pitch = NEWFLAG;
        }

        bool rtkdata_isNew(const location_msgs::msg::RTK &rtkdata)
        {
            return (rtkdata.gpstime > NEWFLAG + 1);
                    // rtkdata.latitude != NEWFLAG &&
                    // rtkdata.longitude != NEWFLAG &&
                    // rtkdata.height != NEWFLAG &&
                    // rtkdata.heading != NEWFLAG &&
                    // rtkdata.pitch != NEWFLAG &&
                    // rtkdata.roll != NEWFLAG);
        }

        // 0 - 原码, 1 - 反码
        double CAN_decode(CMSG raw_data, int lsb, int length, double ratio, double bias, int mode = 0)
        {
            int lsb_byte = lsb / 8;
            int lsb_bit = lsb % 8;
            int msb_bit = (lsb_bit + length - 1) % 8;

            int num_byte;
            if ((lsb_bit + length - 1) / 8 == 0)
                num_byte = 1;
            else
                num_byte = 1 + 1 + (length - (8 - lsb_bit) - (msb_bit + 1)) / 8;
            int msb_byte = lsb_byte - num_byte + 1;
            // printf("num_byte : %d ,lsb_byte : %d, lsb_bit : %d, msb_bit : %d \n",num_byte, lsb_byte, lsb_bit, msb_bit);
            int deviation = 0;
            int data = 0;

            if (num_byte == 1)
            {
                int tmp = 0;
                for (int i = lsb_bit; i <= msb_bit; i++)
                    tmp += (int)pow(2, i);
                data += (int)(raw_data.data[lsb_byte] & tmp);
                data = ratio * data + bias;
                return data;
            }

            for (int byte = lsb_byte; byte > lsb_byte - num_byte; byte--)
            {
                // printf("calculate byte %d : %lf \n", byte, data);
                if (byte == lsb_byte)
                {
                    int tmp = 0;
                    for (int i = lsb_bit; i <= 7; i++)
                        tmp += round(pow(2, i));
                    data += (int)((raw_data.data[byte] & tmp) >> lsb_bit);
                    deviation += (8 - lsb_bit);
                }
                else if (byte == lsb_byte - num_byte + 1)
                {
                    int tmp = 0;
                    for (int i = 0; i <= msb_bit; i++)
                        tmp += round(pow(2, i));
                    data += (int)((raw_data.data[byte] & tmp) << deviation);
                }
                else
                {
                    data += (int)((raw_data.data[byte]) << deviation);
                    deviation += 8;
                }
                // printf("calculate byte %d : %lf \n", byte, data);
            }

            if (mode == 1)
            {
                int tmp = round(pow(2, msb_bit));
                if ((raw_data.data[msb_byte] & tmp) == tmp)
                {
                    data = (-1) * (pow(2, length) - data - 1);
                }
            }

            double output = ratio * data + bias;
            return output;
        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto can_decoder = std::make_shared<can_decode_class>(rclcpp::NodeOptions{});
    can_decoder->main_process();
}


