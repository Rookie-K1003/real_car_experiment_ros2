
#include "CanMsgDefs.h"
#include <iostream>

// 这里按照说明书定义了一些常用函数， 可以自行添加，
// 注意这些函数只修改了预存的数据，
// 并没有修改发送的数据
namespace CanProcess
{

    void send_by_id(unsigned int id, CMSG *data)
    {
        if (all_can_send.find(id) != all_can_send.end())
        {
            all_can_send[id].toDataFrame(data);
        }
    }

    void send_all(CMSG *data)
    {
        int i = 0;

        for (auto &msg_pair : all_can_send)
        {
            auto &msg = msg_pair.second;
            msg.toDataFrame(data + i);
            i++;
        }
    }

    void send_all_safe(send_args *send)
    {
        if (send)
        {
            std::unique_lock<std::mutex> lock(send->datalock);
            send_all(send->data);
        }
    }


    /*---------------------转向类-------------------------*/
    /**
     * 进入自动驾驶模式
    */
    void enterTurn(int car)
    {
        if (car)
        {
            all_can_send[0x65].all_bool_data["VCU_02_WireControlSteerDriveEnable"].data = 1;

        }
        else
        {
            // all_can_send[0x113].all_bool_data["ADS_EPSMode"].data = 2;          
        }
    }
    /**
     * 退出自动驾驶横向（转向）控制
    */
    void exitTurn(int car)
    {
        if (car)
        {
            all_can_send[0x65].all_bool_data["VCU_02_WireControlSteerDriveEnable"].data = 0;
        }
        else
        {
            all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 0;
        }        
    }

    /**
     * 发送方向盘控制信号
    */
    void setTurn(double angle, int car)
    {
        
        if (car)
        {
            all_can_send[0x65].all_double_data["VCU_02_EPS_SteerAngleCmd"].data = 20* angle; // 20241206 乘个传动比测试下
            all_can_send[0x65].all_bool_data["VCU_02_WireControlSteerDriveEnable"].data = 1;
        }
        else
        {
            all_can_send[0x102].all_int_data["HandWheelSteerAngTgt"].data = angle;
            all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 1;
        }        
    }

    /**
     * 获取方向盘转角
    */
    double getTurn(int car)
    {
        double angle;
        if (car)
        {
            angle = all_can_recv[0x75].all_double_data["EPS_SteerAngle"].data;    
        }
        else
        {
            angle =all_can_recv[0x502].all_double_data["HandWheelSteerAngAct"].data;
        }
        return angle;
    }

    /**
     * 获取线控转向是否可被使能状态
    */
    int getEPS_WireControlStatus(int car)
    {
        if (car)
        {
            return all_can_recv[0x75].all_int_data["EPS_WireControlDriveEnable"].data;          
        }
        else
        {
            return 0;
        }        
    }


    /*---------------------驱动制动类-------------------------*/
    // 状态获取
    // double getbrake(int car)
    // {
    //     if (car)
    //     {
    //         return all_can_recv[0x75].all_int_data["EPS_WireControlDriveEnable"].data;          
    //     }
    //     else
    //     {
    //         return 0;
    //     }        
    // }

    // double getWheelSpeed(int car)
    // {
    //     if (car)
    //     {
    //         double avg_wheel_spd, wheel_spd_fl, wheel_spd_fr;
    //         wheel_spd_fl = all_can_recv[0x305].all_double_data["ESC_Wheel_Speed_FL"].data;
    //         wheel_spd_fr = all_can_recv[0x305].all_double_data["ESC_Wheel_Speed_FR"].data;
    //         avg_wheel_spd = (wheel_spd_fl + wheel_spd_fr) * 0.5;
    //         return avg_wheel_spd;          
    //     }
    //     else
    //     {
    //         return 0;
    //     }        
    // }

    // double getESC_Velocity(int car)
    // {
    //     if (car)
    //     {
    //         return all_can_recv[0x305].all_double_data["ESC_Vehicle_Speed"].data;          
    //     }
    //     else
    //     {
    //         return 0;
    //     }        
    // }

    /**
     * 发送线控制动控制信号
    */
    void setBrake(double brake_cmd, int brake_ctrl_enable, int car)
    {
        
        if (car)
        {
            all_can_send[0x63].all_double_data["VCU_02_EHB_BrakeTrqReq"].data = brake_cmd;
            all_can_send[0x63].all_bool_data["VCU_02_WireControlBrakeEnable"].data = brake_ctrl_enable;
        }
        else
        {
            // all_can_send[0x102].all_int_data["HandWheelSteerAngTgt"].data = angle;
            // all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 1;
        }        
    }

     /**
     * 发送驱动电机控制信号
    */
    void setDrive(double drive_cmd, int motor_ctrl_enable, int gear_cmd, int car)
    {
        
        if (car)
        {
            all_can_send[0x64].all_double_data["VCU_02_DriveMotorReqTorque"].data = drive_cmd;
            all_can_send[0x64].all_int_data["VCU_02_GearShiftPositionCmd"].data = gear_cmd;
            all_can_send[0x64].all_bool_data["VCU_02_WireControlDriveEnable"].data = motor_ctrl_enable;
        }
        else
        {
            // all_can_send[0x102].all_int_data["HandWheelSteerAngTgt"].data = angle;
            // all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 1;
        }        
    }

    /**
     * 退出自动驾驶纵向（驱动制动档位）控制
    */
    void exitDrive(int car)
    {
        if (car)
        {
            all_can_send[0x64].all_bool_data["VCU_02_WireControlDriveEnable"].data = 0;
            all_can_send[0x63].all_bool_data["VCU_02_WireControlBrakeEnable"].data = 0;
        }
        else
        {
            // all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 0;
        }        
    }


/*---------------退出所有智驾相关功能---------------*/
    void exitAll(int car)
    {
        exitTurn(car);
        exitDrive(car);
    }

}