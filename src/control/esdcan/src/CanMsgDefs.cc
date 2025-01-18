#include "CanMsgDefs.h"
#include "map"

namespace CanProcess
{
    std::map<unsigned int, CAN_Message> all_can_send;
    std::map<unsigned int, CAN_Message> all_can_recv;

    void initSendMessages()
    {

        unsigned int ids[3] = {0x63, 0x64, 0x65};
        for (auto i : ids)
        {
            all_can_send[i] = CAN_Message(i);
        }

        // ----------all message in message 0x65 , 方向盘控制-----------
        CAN_Message *msg0x65 = &all_can_send[0x65];
        msg0x65->all_bool_data["VCU_02_WireControlSteerDriveEnable"] = SingleMsg_bool("VCU_02_WireControlSteerDriveEnable", 54, 55, 2, 0, 1);
        // msg0x65->all_double_data["VCU_02_EPS_SteerAngleCmd"] = SingleMsg_double("VCU_02_EPS_SteerAngleCmd", 24, 39, 16, -870, 0.1);
        msg0x65->all_double_data["VCU_02_EPS_SteerAngleCmd"] = SingleMsg_double("VCU_02_EPS_SteerAngleCmd", 32, 31, 16, -870, 0.1);
        
        // ----------all message in message 0x63 ，制动控制 -----------
        CAN_Message *msg0x63 = &all_can_send[0x63];
        msg0x63->all_bool_data["VCU_02_WireControlBrakeEnable"] = SingleMsg_bool("VCU_02_WireControlBrakeEnable", 6, 7, 2, 0, 1);
        msg0x63->all_double_data["VCU_02_EHB_BrakeTrqReq"] = SingleMsg_double("VCU_02_EHB_BrakeTrqReq", 32, 27, 12, 0, 1);

        // ----------all message in message 0x64 ，驱动电机和档位控制 -----------
        CAN_Message *msg0x64 = &all_can_send[0x64];
        msg0x64->all_bool_data["VCU_02_WireControlDriveEnable"] = SingleMsg_bool("VCU_02_WireControlDriveEnable", 6, 7, 2, 0, 1);
        msg0x64->all_double_data["VCU_02_DriveMotorReqTorque"] = SingleMsg_double("VCU_02_DriveMotorReqTorque", 32, 27, 12, -3000, 0.1);
        msg0x64->all_int_data["VCU_02_GearShiftPositionCmd"] = SingleMsg_int("VCU_02_GearShiftPositionCmd", 2, 3, 2, 0, 12);      
    }

    void initRecvMessage()
    {
        unsigned int ids[1] = {0x75};
        for (auto i : ids)
        {
            all_can_recv[i] = CAN_Message(i);
        }

        // all message in message 0x75
        CAN_Message *msg0x75 = &all_can_recv[0x75];
        msg0x75->all_int_data["EPS_DriverSteerStatus"] = SingleMsg_int("EPS_DriverSteerStatus", 4, 4, 1);
        msg0x75->all_int_data["EPS_WireControlDriveEnable"] = SingleMsg_int("EPS_WireControlDriveEnable", 5, 7, 3);
        // 存疑，Motorola的数据格式，占了2个字节，直接按layout下的起始和结束位置，是否可以？
        msg0x75->all_double_data["EPS_SteerAngle"] = SingleMsg_double("EPS_SteerAngle", 8, 23, 16, -870, 0.1);

    }

    void canToHuman(CMSG *df, int size)
    {
        for (int i = 0; i < size; i++)
        {
            if (all_can_recv.find(df[i].id) == all_can_recv.end())
                {continue;}

            CAN_Message *temp = &all_can_recv[df[i].id];
            for (auto &msg_pair : temp->all_bool_data)
            {
                auto &msg = msg_pair.second;
                msg.data = decodeMsg<bool>(df[i].data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor);
            }

            for (auto &msg_pair : temp->all_int_data)
            {
                auto &msg = msg_pair.second;
                msg.data = decodeMsg<int>(df[i].data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor);
            }

            for (auto &msg_pair : temp->all_double_data)
            {
                auto &msg = msg_pair.second;
                msg.data = decodeMsg<double>(df[i].data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor);
                //if((df[i].id)==0x500)std::cout<<msg.data<<std::endl;
            }
            
            df[i].id=0;

            
        }
    }

    void CAN_Message::toDataFrame(CMSG *dataframe)
    {
        if (!dataframe)
            {return;}
            
        dataframe->id = id;
        for (int i = 0; i < 8; i++)
        {
            dataframe->data[i] = 0;
        }
        for (auto &msg_pair : all_double_data)
        {
            auto &msg = msg_pair.second;
            // LOG(INFO) << "double message encoded: " << msg.data;
            encodeMsg<double>(msg.data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor, dataframe->data);
        }

        for (auto &msg_pair : this->all_int_data)
        {
            auto &msg = msg_pair.second;
            // LOG(INFO) << "int message encoded: " << msg.data;
            encodeMsg<int>(msg.data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor, dataframe->data);
        }

        for (auto &msg_pair : this->all_bool_data)
        {
            auto &msg = msg_pair.second;
            // LOG(INFO) << "bool message encoded: " << msg.data;
            encodeMsg<bool>(msg.data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor, dataframe->data);
        }
    }
}