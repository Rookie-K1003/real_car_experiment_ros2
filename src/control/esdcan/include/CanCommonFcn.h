#ifndef CANCOMMONFCN_H
#define CANCOMMONFCN_H

#include "CanMsgDefs.h"
// #include <ros/ros.h>
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
            all_can_send[0x65].all_int_data["VCU_02_WireControlSteerDriveEnable"].data = 1;
            all_can_send[0x65].all_bool_data["VCU_02_EPS_SteerAngleCmdValid"].data = 1;
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
            all_can_send[0x65].all_int_data["VCU_02_WireControlSteerDriveEnable"].data = 0;
            all_can_send[0x65].all_bool_data["VCU_02_EPS_SteerAngleCmdValid"].data = 0;
        }
        else
        {
            all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 0;
        }        
    }

    /**
     * @brief 设置车辆转向角度
     *
     * 根据传入的车辆编号和转向角度，设置相应的转向控制数据。
     *
     * @param angle 期望方向盘转向角度，单位：度
     * @param car 车辆编号，0 表示萌健号，1 表示德力车
     */
    void setTurn(double angle, int car)
    {
        
        if (car)
        {
            all_can_send[0x65].all_double_data["VCU_02_EPS_SteerAngleCmd"].data = angle;
            all_can_send[0x65].all_int_data["VCU_02_WireControlSteerDriveEnable"].data = 1;
            all_can_send[0x65].all_bool_data["VCU_02_EPS_SteerAngleCmdValid"].data = 1;
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
    int getEPSWireControlStatus()
    {     
        return all_can_recv[0x75].all_int_data["EPS_WireControlDriveEnable"].data;          
    }
    /*
     * 获取转向驾驶员接管状态
    */
    int getEPSDriverSteerStatus()
    {
        return all_can_recv[0x75].all_int_data["EPS_DriverSteerStatus"].data;          
    }

    /*---------------------驱动制动类-------------------------*/

    double getWheelSpeed(int car)
    {
        if (car)
        {
            double avg_wheel_spd, wheel_spd_fl, wheel_spd_fr;
            // CAN信号获得的单位时km/h，转换为m/s
            wheel_spd_fl = all_can_recv[0x305].all_double_data["ESC_Wheel_Speed_FL"].data;
            wheel_spd_fr = all_can_recv[0x305].all_double_data["ESC_Wheel_Speed_FR"].data;
            avg_wheel_spd = (wheel_spd_fl + wheel_spd_fr) * 0.5 / 3.6; // 单位m/s
            return avg_wheel_spd;          
        }
        else
        {
            return 0;
        }        
    }

    double getESCVelocity(int car)
    {
        if (car)
        {
            return all_can_recv[0x305].all_double_data["ESC_Vehicle_Speed"].data / 3.6; // 单位m/s        
        }
        else
        {
            return 0;
        }        
    }

    /**
     * 发送线控制动控制信号
    */
    void setBrake(double brake_cmd, int brake_ctrl_enable, int car)
    {
        
        if (car)
        {
            all_can_send[0x63].all_double_data["VCU_02_EHB_BrakeTrqReq"].data = brake_cmd;
            all_can_send[0x63].all_int_data["VCU_02_WireControlBrakeEnable"].data = brake_ctrl_enable;
            all_can_send[0x63].all_int_data["VCU_02_EHB_BrakeTrqReqValid"].data = brake_ctrl_enable;
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
            all_can_send[0x64].all_int_data["VCU_02_WireControlDriveEnable"].data = motor_ctrl_enable;
            all_can_send[0x64].all_int_data["VCU_02_GearShiftPositionCmdValid"].data = motor_ctrl_enable;            
            all_can_send[0x64].all_int_data["VCU_02_DriveMotorReqTorqueValid"].data = motor_ctrl_enable;
            all_can_send[0x64].all_int_data["VCU_02_GearShiftPositionCmd"].data = gear_cmd;
            all_can_send[0x64].all_double_data["VCU_02_DriveMotorReqTorque"].data = drive_cmd;
        }
        else
        {
            // all_can_send[0x102].all_int_data["HandWheelSteerAngTgt"].data = angle;
            // all_can_send[0x102].all_bool_data["SteerEnCtrl"].data = 1;
        }        
    }

    // --------------------- 制动部分反馈 ---------------------
    /*
     * 获取EHB实际制动扭矩（轮端）
    */
    double getEHBBrakeTorque()
    {
        
        return all_can_recv[0x72].all_double_data["EHB_BrkTrqAct"].data;
    }

    /*
     * 获取EHB驾驶员接管状态
    */
    int getEHBDriverBrakeStatus()
    {
        return all_can_recv[0x72].all_int_data["EHB_DriverBrkStatus"].data;          
    }

    /*
     * 获取线控制动是否可被使能状态
    */
    int getEHBWireControlBrakeEnable()
    {     
        return all_can_recv[0x72].all_int_data["EHB_WireControlBrakeEnable"].data;          
    }

    // ---------------- 驱动部分反馈 ----------------
    /*
     * 获取驱动电机实际扭矩（轮端）
    */
    double getMotorPresentTorque()
    {
        return all_can_recv[0x74].all_double_data["VCU_MotorPresentTorque"].data;          
    }

    /*
     * 获取线控驱动是否可被使能状态
    */
    int getWireControlDriveEnable()
    {     
        return all_can_recv[0x74].all_int_data["VCU_WireControlDriveEnable"].data;          
    }
    /*
     * 获取驾驶员油门接管状态
    */
    int getDriverThrottleStatus()
    {
        return all_can_recv[0x74].all_int_data["VCU_DriverThrottleStatus"].data;          
    }
    /*
     * 获取当前档位
    */
    int getActualGearShiftPosition()
    {
        return all_can_recv[0x74].all_int_data["VCU_ActualGearShiftPosition"].data;          
    }

    /**
     * 退出自动驾驶纵向（驱动制动档位）控制
    */
    void exitDrive(int car)
    {
        if (car)
        {
            all_can_send[0x64].all_int_data["VCU_02_WireControlDriveEnable"].data = 0;
            all_can_send[0x63].all_int_data["VCU_02_WireControlBrakeEnable"].data = 0;
            all_can_send[0x64].all_int_data["VCU_02_DriveMotorReqTorqueValid"].data = 0;
            all_can_send[0x64].all_int_data["VCU_02_GearShiftPositionCmdValid"].data = 0;
            all_can_send[0x63].all_int_data["VCU_02_EHB_BrakeTrqReqValid"].data = 0;
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
#endif
