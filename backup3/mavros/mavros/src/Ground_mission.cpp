/***************************************************************************************************************************
无人蜂群 2019
@file:/home/ubuntu/ws_dji/src/Onboard-SDK-ROS/dji_sdk_demo/src/mission_thrower.cpp
@author: Zzy
@date:2019.04.07
@date:2019.04.19
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>

//msg 头文件
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>



//DJI SDK 头文件
//#include <dji_sdk/Data_Throw_target.h>
//#include <dji_sdk/Data_Drone_State.h>
#include <iomanip>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros/Data_Drone_State.h>
#include <mavros/Data_Throw_target.h>



#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全局变量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.1415926535898

//-----------------------------------------电台通讯相关----------------------------------------------------
mavros::Data_Drone_State received_mission;
mavros::Data_Throw_target Wsend_target_data;

//##########################
//定义要接受的消息的类型及名称
sensor_msgs::NavSatFix current_state;

int drop_flag;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vitrual_data_callback(const std_msgs::UInt16::ConstPtr& msg)
{
    drop_flag = msg->data;

    switch (drop_flag)
    {

    case 1:
        Wsend_target_data.Accepted_ID = 0x03;          // 接收此指令的 投掷机ID (0x1-0x8, 0xF全体返航)
        Wsend_target_data.flag_Return = 0;          // 返航标志	0-不返航   1-返航
        Wsend_target_data.target_ID = 5;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.target_type = 1;          // 投掷目标类型 /普通-1 /一般-2 /重点-3
        Wsend_target_data.target_Num = 0;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.latitude = 39.653699;     //纬度 latitude   -- 取小数后6位
        Wsend_target_data.longitude = 116.133255;   //经度 longitude  -- 取小数后6位
        Wsend_target_data.enterangle = 90;          //角度 对于重点目标 需要三架<1min内完成的指标// ((float) buffer_rx[8+18])*256/360;
        Wsend_target_data.partner1 = 0x06;
        Wsend_target_data.partner2 = 0x08;          //两架协同完成任务的无人机编号

        break;

    case 2:
        Wsend_target_data.Accepted_ID = 0x03;          // 接收此指令的 投掷机ID (0x1-0x8, 0xF全体返航)
        Wsend_target_data.flag_Return = 0;          // 返航标志	0-不返航   1-返航
        Wsend_target_data.target_ID = 7;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.target_type = 2;          // 投掷目标类型 /普通-1 /一般-2 /重点-3
        Wsend_target_data.target_Num = 1;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.latitude = 39.653699;     //纬度 latitude   -- 取小数后6位
        Wsend_target_data.longitude = 116.133255;   //经度 longitude  -- 取小数后6位
        Wsend_target_data.enterangle = 145;          //角度 对于重点目标 需要三架<1min内完成的指标// ((float) buffer_rx[8+18])*256/360;
        Wsend_target_data.partner1 = 0x07;
        Wsend_target_data.partner2 = 0x04;          //两架协同完成任务的无人机编号

        break;

    case 3:
        Wsend_target_data.Accepted_ID = 0x03;          // 接收此指令的 投掷机ID (0x1-0x8, 0xF全体返航)
        Wsend_target_data.flag_Return = 0;          // 返航标志	0-不返航   1-返航
        Wsend_target_data.target_ID = 7;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.target_type = 2;          // 投掷目标类型 /普通-1 /一般-2 /重点-3
        Wsend_target_data.target_Num = 0;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.latitude = 39.653699;     //纬度 latitude   -- 取小数后6位
        Wsend_target_data.longitude = 116.133255;   //经度 longitude  -- 取小数后6位
        Wsend_target_data.enterangle = 145;          //角度 对于重点目标 需要三架<1min内完成的指标// ((float) buffer_rx[8+18])*256/360;
        Wsend_target_data.partner1 = 0x07;
        Wsend_target_data.partner2 = 0x04;          //两架协同完成任务的无人机编号

        break;


    case 777:
        Wsend_target_data.Accepted_ID = 0x03;          // 接收此指令的 投掷机ID (0x1-0x8, 0xF全体返航)
        Wsend_target_data.flag_Return = 1;          // 返航标志	0-不返航   1-返航
        Wsend_target_data.target_type = 0;          // 投掷目标类型 /普通-1 /一般-2 /重点-3
        Wsend_target_data.target_Num = 0;           // 10个投掷目标的编号 1-10 根据被识别的顺序
        Wsend_target_data.latitude = 0;     //纬度 latitude   -- 取小数后6位
        Wsend_target_data.longitude = 0;   //经度 longitude  -- 取小数后6位
        Wsend_target_data.enterangle = 0;          //角度 对于重点目标 需要三架<1min内完成的指标// ((float) buffer_rx[8+18])*256/360;
        Wsend_target_data.partner1 = 0;
        Wsend_target_data.partner2 = 0;          //两架协同完成任务的无人机编号

        break;
    }
}


void WaitSolveFromData_Drone_State_callback(const mavros::Data_Drone_State::ConstPtr& msg)
{   received_mission = *msg;

    cout << endl << "  ----------  Received Drone state From ROS->serial.cpp ---------  "<<endl;
    cout << "From Drone 0x" << dec << received_mission.Drone_ID << " -- "<< endl;
    cout << "Drone_state: " << dec << received_mission.Drone_state << ", /0-performing Task /111-waiting New Task /777-Forced return"<<endl;
    cout << "target_ID: " << received_mission.target_ID << endl;
    cout << "target_type: " << received_mission.target_type << endl;
    cout << "target_Num: " << received_mission.target_Num << endl;


}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_virtual_Ground");
    ros::NodeHandle nh("~");

    //需要更新的状态  --通过 ROS 接收 给 mission_Ground.cpp
    ros::Subscriber received_Throw_state_Sub = nh.subscribe<mavros::Data_Drone_State>("/Mission_Data_Drone_State_renew", 10, WaitSolveFromData_Drone_State_callback);
    ros::Subscriber vitrual_data_Sub = nh.subscribe<std_msgs::UInt16>("/vitrual_data", 10, vitrual_data_callback);

    //需要通过 通信模块 发布出的信息
    //--通过 ROS 发送 mission_Ground.cpp 的话题
    //--包括 \接收的设备 \发布信信息[1-返航 2-投掷目标(投掷目标类型 投掷目标编号 本次投掷次数)]
    ros::Publisher WaitSendtoData_Throw_target_Pub = nh.advertise<mavros::Data_Throw_target>("/WaitSendtoData_Throw_target", 10);

    ros::Rate loop_rate(20);                                   //数据状态更新频率 [20Hz]

    cout <<"rostopic pub /vitrual_data std_msgs/UInt16 \"data: 0\" "<< endl;
    cout << " data: /1-target_type=1  /2-target_type=2  /777-ForcedBack " << endl;


    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        if (drop_flag >0)
        {

            cout << endl << "  ----------  Send Target state to ROS->serial.cpp ---------  "<<endl;
            cout << "To Drone 0x" << dec << (int)Wsend_target_data.Drone_ID << endl;
            cout << "flag_Return: " << dec << (int)Wsend_target_data.flag_Return << ", /0-No return back Task /1-return back" << endl;
            cout << "target_ID: " << dec << (int)Wsend_target_data.target_ID;
            cout << "target_type: " << dec << (int)Wsend_target_data.target_type << ", /0 - return back /1 /2 /3" << endl;
            cout << "target_Num: "  << dec << (int)Wsend_target_data.target_Num << endl;
            cout << "latitude: "  << fixed << setprecision(9) << Wsend_target_data.latitude << endl;
            cout << "longitude: "  << fixed << setprecision(9) << Wsend_target_data.longitude << endl;
            cout << "enterangle: "  << fixed << setprecision(2) << Wsend_target_data.enterangle << endl;
            cout << "partner 1&2: "  << dec << Wsend_target_data.partner1 << " & " << Wsend_target_data.partner2 <<endl;

            WaitSendtoData_Throw_target_Pub.publish(Wsend_target_data);

            drop_flag = 0;
        }

        loop_rate.sleep();
    }

    return 0;
}


