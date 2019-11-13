/* 串口通信 程序 银河
 * 地面站
 * Author:  Zzy
 * Date:    2019.04.16
 *  *  * Date:    2019.04.19
 * Function:
 * 地面站 <--> 投掷机
 *          0.投掷机
 *          1.串口通信模块数据接收&回应
 *              1.1 发送投掷机0x03-0x08的  投掷目标信息
 *              1.2 发送投掷机0x03-0x08的  中断返航指令
 *          2.ROS节点与串口通信模块的交互
 *              2.1 将接收到的投掷目标信息发布
 *              2.2 重新装载沙包后，发布信息告诉地面站0x0a
 *              2.3 投掷时，发布投掷信息，使地面站0x0a 记录投掷信息
 */

//yinhe_serial.cpp

#include <ros/ros.h>

#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <cstring>

#include <fstream>
#include <cstdlib>
#include <stdlib.h>

//topic msg头文件
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
//#include <dji_sdk/Data_Throw_target.h>       //ROS -> serial    投掷机 需要 接收&执行 的指令信息
//#include <dji_sdk/Data_Drone_State.h>        //serial -> ROS    地面站 接收到的 投掷机的 状态信息



#include <iomanip>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros/Data_Drone_State.h>
#include <mavros/Data_Throw_target.h>





using namespace std;

//##################################################            全局变量         ############################################################//
//----------------------------------------          无线模块相关          ----------------------------------------//
//通信模块配置 相关帧头
//上电节点配置
const uint8_t CONFIG_Header_0 = 0xEB;               //帧头0
const uint8_t CONFIG_Header_1 = 0x90;               //帧头1
const uint8_t CONFIG_ID =       0x0a;               //通信模块编号 | 与飞机编号相同   0x01&0x02_侦察机    0x03-0x08_投掷机    0x0a_地面站

//接收数据
const uint8_t Data_HEADER_0	= 0X9F;                 //帧头0
const uint8_t Data_HEADER_1	= 0XE4;                 //帧头1

//自定义数据帧头帧尾
const uint8_t CustomData_Header_0	= 0X31;         //帧头0
const uint8_t CustomData_Header_1	= 0X6B;         //帧头1

//统计连接在内的模块
char16_t Connected_Num = 0x0001 << CONFIG_ID;       //统计连接在系统内的模块
int connected_Count = 0;                            //统计连接的模块计数

uint8_t Rbuffer_rx[512];                            //串口接收Raw  20Hz读取频率
size_t  Rn;                                         //串口接收Raw 字节数

int flag_find;                                      //帧头寻找 Flag
size_t n_last, n_present;                           //指针*
uint8_t buffer_rx_precent[11][128];                 //Rbuffer_rx 分割帧后   01-0a
int     n_rx_precent[11];                           //buffer_rx_precent[x] 中的有效字节数

int flag_complete;                                  //包完整性判断Flag
uint8_t buffer_rx[128];                             //串口接收 一个完整的自定义数据包
size_t  n;                                          //串口接收 一个完整的自定义数据包 字节数

uint8_t buffer_temp[512];                           //临时数组

//串口待发送信息缓存 20Hz 每次只发送一条信息
//后期需修改!!!!!!
uint8_t buffer_tx[16][256];                             //串口发送 缓存
size_t  n_tx[16];                                       //串口发送 缓存 对应字节数 0-15  0x0-0xF
int  n_tx_current_flag = -1;                              //串口发送 缓存 标志 - -1 没有

uint8_t Wbuffer_tx[256];                            //串口实际发送 [自定义 帧头*2 & 数据位*1 & 数据*n & 校验位*1]
size_t  Wn_tx;                                      //串口实际发送 字节数

//----------------------------------------        ROS&自定义信息相关       ----------------------------------------//
uint8_t sendID;                                     //接收到的信息的发送端ID
uint8_t Order_num;                                  //指令编号  -- 用于通信应答!!!! 待完成   [l3-0]

int Drone_ID;       //投掷机 编号
int Accepted_ID;   //接收机 编号
int Drone_state;    //##本机状态	/111 - 0x0a接收到此指令后，即得知该机可以接收新的目标
//          /777    被强制返航   /0	本机有正在执行的任务
int target_ID;     // 投掷目标编号 1-10 根据被识别到的顺序
int target_type;   //投掷目标 类型
int target_Num;    //对应投掷目标已经投掷次数	/无人机投掷时发布	/无人机被强制返航时发布




mavros::Data_Throw_target Wsend_target_data;       //等待发送 给 投掷机 的 目标&指令 (ROS -> 通信模块)
mavros::Data_Drone_State  Received_target_data;    //接收到的 投掷机 的状态 (ROS -> 通信模块)




//////////////////////////////////////////
///////////////////第一处改动///////////////////
///////////////////////////////////////////////////////
//自己定义的gps的消息类型的变量
sensor_msgs::NavSatFix current_state;
//////////////////////////////////////////
///////////////////第一处改动///////////////////
///////////////////////////////////////////////////////
//自己定义的gps的消息类型的变量






//##################################################            其他函数声明         ############################################################//
void show_connectedDevice();



//#################第二处改动#######################            回调函数my       ############################################################//
//##########################
//回调函数中的前两个冒号中的内容要需要与消息的类型对应
void state_cb(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    current_state = *msg;
}
//##########################

//##################################################            回调函数         ############################################################//
void WaitSendtoData_Throw_target_callback(const mavros::Data_Throw_target::ConstPtr& msg)
{
    Wsend_target_data = *msg;

    int Received_msg_ID = Wsend_target_data.Accepted_ID;       // 接收此指令的 投掷机ID (0x1-0x8, 0xF全体返航)
    int flag_Return = Wsend_target_data.flag_Return;        // 返航标志	0-不返航   1-返航

    int target_ID = Wsend_target_data.target_ID;          // 10个投掷目标的编号 1-10 根据被识别的顺序
    int target_type = Wsend_target_data.target_type;          // 10个投掷目标的编号 1-10 根据被识别的顺序
    int target_Num = Wsend_target_data.target_Num;        // 投掷目标类型 /普通-1 /一般-2 /重点-3

    double target_latitude = Wsend_target_data.latitude;    //纬度 latitude   -- 取小数后6位
    double target_longitude = Wsend_target_data.longitude;  //经度 longitude  -- 取小数后6位

    float target_angle = Wsend_target_data.enterangle;      //角度 对于重点目标 需要三架<1min内完成的指标// ((float) buffer_rx[8+18])*256/360;
    int cooperation_Drone1 = Wsend_target_data.partner1;
    int cooperation_Drone2 = Wsend_target_data.partner2;    //两架协同完成任务的无人机编号
    //  uint8_t Order_num;                                  //指令编号  -- 用于通信应答!!!! 待完成   [l3-0]

    n_tx_current_flag = n_tx_current_flag+1;

    switch (Received_msg_ID)
    {//全体返航 0xF
    case 15:
        /*地面站 发送 给 投掷机 的 指令格式
         * 包括 投掷123目标/单机返航/全体返航
         *
         * buffer[0] |char_8     [h7-l0]
         *   [h7-4  target_ID          10个投掷目标的编号 1-10 根据被识别的顺序
         *   /h3-l0 Received_msg_ID     接收指令的无人机(0x1-0x8, 0xF全体返航/已提前判断了)]
         * buffer[1] |char_8     [h7-l0]
         *   [h7-6  target_type         投掷目标类型(0b01_1,0b10_2,0b11_3,0b00_返航)
         *    h5-4  target_Num          投掷目标已投掷次数s
         *    h3-l0 Order_num/n_tx_current_flag  指令编号      *用于握手应答判断
         * buffer[2] 无意义 保留
         */

        //buffer[0] |char_8     [h7-l0]
        //         *   [h7-4  target_ID          10个投掷目标的编号 1-10 根据被识别的顺序
        //         *   /h3-l0 Received_msg_ID     接收指令的无人机(0x1-0x8, 0xF全体返航/已提前判断了)]
        buffer_tx[n_tx_current_flag][0] =  0x0F;

        //buffer[1] |char_8     [h7-l0]
        //         *   [h7-6  target_type         投掷目标类型(0b01_1,0b10_2,0b11_3,0b00_返航)
        //         *    h5-4  target_Num          投掷目标已投掷次数
        //         *    h3-l0 Order_num/n_tx_current_flag  指令编号      *用于握手应答判断
        buffer_tx[n_tx_current_flag][1] = (0x0F & ((char)n_tx_current_flag));    // 指令编号	*用于握手应答判断

        //buffer[2] 此时无意义
        buffer_tx[n_tx_current_flag][2] = 0x66;

        n_tx[n_tx_current_flag] = 3;

        break;

        //侦察机 0x01 & 0x02
    case 1:
    case 2:


        break;

    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
        /*地面站 发送 给 投掷机 的 指令格式
         * 包括 投掷123目标/单机返航/全体返航
         *
         * buffer[0] |char_8     [h7-l0]
         *   [h7-4  target_ID          10个投掷目标的编号 1-10 根据被识别的顺序
         *   /h3-l0 Received_msg_ID     接收指令的无人机(0x1-0x8, 0xF全体返航/已提前判断了)]
         * buffer[1] |char_8     [h7-l0]
         *   [h7-6  target_type         投掷目标类型(0b01_1,0b10_2,0b11_3,0b00_返航)
         *    h5-4  target_Num          投掷目标已投掷次数
         *    h3-l0 Order_num/n_tx_current_flag  指令编号      *用于握手应答判断
         * buffer[2-5] |小数点后6位  --  精度0.1m
         *   [投掷点 纬度] target_latitude
         * buffer[6-9]|小数点后6位  --  精度0.1m
         *   [投掷点 经度] target_longitude
         * buffer[10] |char_8
         *   [攻击进入角] |360<-->256 范围内转化
         *      target_angle  -> char
         * buffer[11] |char_8    [h7-l0]
         *   [两架协同攻击无人机编号] (h7-4)&(3-l0)
         *   cooperation_Drone1&2
         */

        //buffer[0] |char_8     [h7-l0]
        //         *   [h7-4  target_ID          10个投掷目标的编号 1-10 根据被识别的顺序
        //         *   /h3-l0 Received_msg_ID     接收指令的无人机(0x1-0x8, 0xF全体返航/已提前判断了)]
        buffer_tx[n_tx_current_flag][0] =  (( 0x0F & ((char)target_ID)) << 4) |( 0x0F & ((char)Received_msg_ID));

        //buffer[1] |char_8     [h7-l0]
        //         *   [h7-6  target_type         投掷目标类型(0b01_1,0b10_2,0b11_3,0b00_返航)
        //         *    h5-4  target_Num          投掷目标已投掷次数
        //         *    h3-l0 Order_num/n_tx_current_flag  指令编号      *用于握手应答判断
        buffer_tx[n_tx_current_flag][1] =  (( 0x03 & ((char)target_type)) << 6) | (( 0x03 & ((char)target_Num)) << 4) | (0x0F & ((char)n_tx_current_flag));    // 指令编号	*用于握手应答判断

        //buffer[2-5] |小数点后6位  --  精度0.1m
        //*   [投掷点 纬度] target_latitude
        double temp_double = target_latitude;
        for (int i=2; i<6; i++)
        {
            buffer_tx[n_tx_current_flag][i] = (int) (temp_double);
            temp_double = (temp_double - buffer_tx[n_tx_current_flag][i]) * 100;
        }

        //buffer[6-9]|小数点后6位  --  精度0.1m
        //*   [投掷点 经度] target_longitude
        temp_double = target_longitude;
        for (int i=6; i<10; i++)
        {
            buffer_tx[n_tx_current_flag][i] = (int) (temp_double);
            temp_double = (temp_double - buffer_tx[n_tx_current_flag][i]) * 100;
        }

        //buffer[10] |char_8
        //*   [攻击进入角] |360<-->256 范围内转化
        //*      target_angle  -> char
        buffer_tx[n_tx_current_flag][10] = (int) (target_angle/360*256);

        //buffer[11] |char_8    [h7-l0]
        //*   [两架协同攻击无人机编号] (h7-4)&(3-l0)
        //*   cooperation_Drone1&2
        buffer_tx[n_tx_current_flag][11] =  (( 0x0F & ((char)cooperation_Drone1)) << 4) | (0x0F & ((char)cooperation_Drone2));

        n_tx[n_tx_current_flag] = 12;

        break;
    }

}

//##################################################            主函数         ############################################################//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_Ground");
    ros::NodeHandle nh("~");                                     //创建句柄

    //需要通过 通信模块 发布出的信息
    //--通过 ROS 接收 mission_Ground.cpp 的话题
    //--包括 \接收的设备 \发布信信息[1-返航 2-投掷目标(投掷目标类型 投掷目标编号 本次投掷次数)]
    ros::Subscriber WaitSendtoData_Throw_target_Sub = nh.subscribe<mavros::Data_Throw_target>("/WaitSendtoData_Throw_target", 10, WaitSendtoData_Throw_target_callback);

    //需要更新的状态  --通过 ROS 发布 给 mission_Ground.cpp
    ros::Publisher received_Throw_state_Pub = nh.advertise<mavros::Data_Drone_State>("/Mission_Data_Drone_State_renew", 10);

    ros::Rate loop_rate(20);                                   //数据状态更新频率 [20Hz]

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          串口 打开           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    //串口serial设置&开启
    serial::Serial sp;                                          //创建一个serial类
    serial::Timeout to = serial::Timeout::simpleTimeout(10);   //创建timeout
    sp.setPort("/dev/ttyUSB0");                                 //设置要打开的串口名称
    sp.setBaudrate(115200);                                     //设置串口通信的波特率
    sp.setTimeout(to);                                          //串口设置timeout
    //打开串口
    try
    {    sp.open();
    }
    catch(serial::IOException& e)
    {   ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if(sp.isOpen())
    {   ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {   return -1;
    }
    //----------------------------------------          串口打开 完成         --------------------------------------------------//

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块 初始配置           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    // 串口打开后，配置模块 地址 EB 90 + 飞机编号
    Wbuffer_tx[0] = CONFIG_Header_0;     //配置帧头 0xEB
    Wbuffer_tx[1] = CONFIG_Header_1;     //配置帧头 0x90
    Wbuffer_tx[2] = CONFIG_ID;           //本机编号 0x03-0x09
    cout<<"Send_Configuration_Set: ";
    for(int i=0; i<3; i++)
    {
        cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
    }
    cout << endl << "Please Wait!!" << endl << endl;

    int flag_config = 1;
    while(flag_config)
    {
        ros::spinOnce();

        sp.write(Wbuffer_tx, 3);
        usleep(25000);              //百万分之一秒        //5,0000 * 0.000 001 = 0.05
        //获取缓冲区内的字节数
        Rn = sp.available();
        if(Rn!=0)
        {
            Rn = sp.read(Rbuffer_rx, Rn);  //读出数据
            if (Rn==4)   //配置返回信息
            {
                if (Rbuffer_rx[0]==CONFIG_Header_0 && Rbuffer_rx[1]==CONFIG_Header_1 && Rbuffer_rx[2]==0x00 && Rbuffer_rx[3]==CONFIG_ID )
                {
                    flag_config = 0;
                    cout<<"Configuration transreceiver Sucessed!  Info  -- Start connected"<<endl;
                }
                else
                {
                    flag_config = 1;
                    cout<<"Configuration transreceiver Failed!  Info  -- n=4 Failed"<<endl;
                }
            } // End{ if (Rn==4)
            else
            {
                if (Rbuffer_rx[0]==Data_HEADER_0 && Rbuffer_rx[1]==Data_HEADER_1 && Rbuffer_rx[2]==CONFIG_ID )
                {
                    flag_config = 0;
                    cout<<"Configuration transreceiver Sucessed!  Info  -- Already connected"<<endl;
                }
                else
                {
                    flag_config = 1;
                    cout<<"Configuration transreceiver Failed!  Info  -- n<>4 Failed"<<endl;
                }
            }
            //接收信息打印
            cout<<"Set_Transceiver_Result: ";
            for(int i=0; i<Rn; i++)
            {
                cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
            }
            cout << endl << endl;
        }   // if(Rn!=0)
        loop_rate.sleep();
    }   // while(flag_config)
    //----------------------------------------          模块 初始配置 完成         --------------------------------------------------//

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 串口读取&发送&处理           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    ofstream outfile;

    //部分变量&缓存数组初始化
    for(int i=1; i<11; i++)
    {
        buffer_rx_precent[i][0] = 0;
        n_rx_precent[i] = 0;
    }
    n_tx_current_flag = -1;

    while(ros::ok())
    {
        ros::spinOnce();
        // 获取缓冲区内的字节数
        Rn = sp.available();

        if(Rn!=0)
        {
            Rn = sp.read(Rbuffer_rx, Rn);      // 读取串口数据
            // 输出串口收到的数据
            cout << endl << endl << "-------------this is the new data-----------" <<endl;
            cout << "Rn = " << Rn << endl << "Receive_Data: ";
            for(int i=0; i<Rn; i++)
            {
                cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
            }
            cout << endl;

            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          分割 Rbuffer_rx           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
            n_present = 0;
            n_last = n_present;
            sendID = 0x00;
            Connected_Num = 0x0001 << CONFIG_ID;
            connected_Count = 0;

            //寻找最开始的 EB90 如果 不是0 则输出 error 到txt文件!!
            flag_find = 0;
            while (flag_find ==0)
            {
                if (n_present+2==Rn)
                {
                    flag_find = 1;
                    n_present = Rn;
                    //没有找到帧头 到最后了
                }
                else if (Rbuffer_rx[n_present+0]==Data_HEADER_0 && Rbuffer_rx[n_present+1]==Data_HEADER_1 && Rbuffer_rx[n_present+2]==CONFIG_ID )
                {
                    flag_find = 1;
                }
                else
                {
                    flag_find = 0;
                    n_present = n_present+1;
                }
            }
            //cout << "n_present init:" << n_present << ", n_last inti:" << n_last << endl;

            // 如果 EB 90 CONFIG_ID 不是 Rbuffer_rx[0][1][2] 则输出 error 到txt文件!!
            if (n_present != 0)
            {
                outfile.open("/home/ubuntu/serial_error/Rbuffer_rx.txt",ios::app);
                outfile << endl <<"-- EB 90 -- is not Rbuffer_rx[0][1]" << endl;
                outfile << "Rbuffer_rx : ";
                for(int i=0; i<Rn; i++)
                {
                    outfile << hex << (Rbuffer_rx[i] & 0xff) << " ";
                }
                outfile << endl;
                outfile.close();
            }

            //进行分割
            while( n_present<Rn )
            {
                n_last = n_present;         //下个包的开头
                n_present = n_present+1;    //寻找下下一个包的开头

                flag_find = 0;              //flag复位
                while (flag_find ==0 )
                {
                    if ( n_present+2 == Rn)
                    {
                        flag_find = 1;
                        n_present = Rn;     //没有找到帧头 到最后了
                    }
                    else if (Rbuffer_rx[n_present+0]==Data_HEADER_0 && Rbuffer_rx[n_present+1]==Data_HEADER_1 && Rbuffer_rx[n_present+2]==CONFIG_ID )
                    {
                        flag_find = 1;      //下下一个包的开头
                    }
                    else
                    {
                        flag_find = 0;
                        n_present = n_present+1;
                    }
                }

                sendID = Rbuffer_rx[n_last+3];
                cout <<  "n_last :" << n_last << ", n_present :" << n_present <<endl;
                cout << "Data From [" << dec << (int)sendID << "]";

                n = n_present - n_last;
                if (n > 8)
                {   //数据复制到对应的ID后
                    memcpy(buffer_rx_precent[sendID]+n_rx_precent[sendID], Rbuffer_rx+n_last+8, n-8);
                    n_rx_precent[sendID] = n_rx_precent[sendID] + n-8;

                    cout << ", n =" << dec << n << ", Receive_Data: ";
                    for(int i=n_last; i<n_present; i++)
                    {
                        cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
                    }

                    outfile.open("/home/ubuntu/serial_error/Received_Data.txt",ios::app);
                    outfile << endl <<"n_rx_precent[" << dec << (int)sendID << "] , n=" << (int)n_rx_precent[sendID] << endl;
                    for(int i=0; i<n_rx_precent[sendID]; i++)
                    {
                        outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();
                }
                else if (n < 8)
                { // !!!!串口读取有错误
                    outfile.open("/home/ubuntu/serial_error/Rbuffer_rx.txt",ios::app);
                    outfile << endl <<"-- n < 8 -- Rbuffer_rx error -- ID: " << dec <<(int)sendID << endl;
                    outfile << "Rbuffer_rx : ";
                    for(int i=0; i<Rn; i++)
                    {
                        outfile << hex << (Rbuffer_rx[i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();
                }
                else
                { // n==8 !!!!上一回的数据可能有错误
                    if (n_rx_precent[sendID] >0)
                    {
                        outfile.open("/home/ubuntu/serial_error/Rbuffer_rx.txt",ios::app);
                        outfile << endl <<" -- n_rx_precent[sendID] >0 error -- ID: " << dec << (int)sendID << endl;
                        outfile << "n_rx_precent : ";
                        for(int i=0; i<n_rx_precent[sendID]; i++)
                        {
                            outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                        }
                        outfile << endl;
                        outfile.close();
                    }
                }
                cout << endl;
                //----------------------------------------          分割 Rbuffer_rx  完成         --------------------------------------------------//


                if (sendID != 0x00) //统计已连接的模块
                {
                    Connected_Num = (Connected_Num | (0x0001 << sendID ));
                }
                connected_Count = connected_Count+1;
            }

            show_connectedDevice();     //显示已连接模块数量和编号

        }//if(Rn!=0)
        else
        {
            sendID = 0x00;  //记录已连接模块ID清零
            cout << endl << endl << "-------------this is the new data-----------" << endl << "Serial Received NO Data & No device connected !" <<endl;
        }

        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 包处理&给出一个完整包           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
        //n_rx_precent[sendID]
        //buffer_rx_precent[sendID][i]
        for (sendID = 1; sendID<11; sendID++)
        {
            if (n_rx_precent[sendID]>0)
            {   //找出包
                //找到开头帧 316B
                n_present = 0;

                flag_find = 0;
                while (flag_find ==0)
                {
                    if (n_present+1 == n_rx_precent[sendID])
                    {
                        flag_find = 1;
                        n_present = n_rx_precent[sendID];
                        //没有找到帧头 到最后了
                    }
                    else if (buffer_rx_precent[sendID][n_present+0]==CustomData_Header_0 && buffer_rx_precent[sendID][n_present+1]==CustomData_Header_1)
                    {
                        flag_find = 1;
                    }
                    else
                    {
                        flag_find = 0;
                        n_present = n_present+1;
                    }
                }

                // 如果 31 6B 不是 buffer_rx_precent[ID][0][1] 则输出 error 到txt文件!!
                if (n_present != 0)
                {
                    outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                    outfile << endl <<"-- 316B -- is not buffer_rx_precent[" << dec << (int)sendID <<"][0][1] -- Error!" << endl;
                    outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                    for(int i=0; i<n_rx_precent[sendID]; i++)
                    {
                        outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();

                    //前移到[0]
                    if (n_rx_precent[sendID] - n_present >0)
                    {
                        memcpy(buffer_temp, buffer_rx_precent[sendID]+n_present, n_rx_precent[sendID] - n_present);
                        memcpy(buffer_rx_precent[sendID], buffer_temp, n_rx_precent[sendID] - n_present);

                        n_rx_precent[sendID] = n_rx_precent[sendID] - n_present;
                        n_present = 0;
                    }
                    else if (n_rx_precent[sendID] - n_present == 0)
                    {
                        n_rx_precent[sendID] = 0;
                        n_present = 0;
                    }

                    outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                    outfile  <<"After Move" << endl;
                    outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                    for(int i=0; i<n_rx_precent[sendID]; i++)
                    {
                        outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();

                }//End if (n_present != 0)

                flag_complete = 0;

                if (n_rx_precent[sendID] > 2)
                {
                    //判断包的长度是否大于  0x20 /0d32
                    if (buffer_rx_precent[sendID][2] <0x21)
                    {//判断包的长度不大于  0x20 /0d32    人为定义

                        //判断是否是一个完整的包
                        if (buffer_rx_precent[sendID][2] + 4 <= n_rx_precent[sendID])
                        {  //是一个完整的包

                            n = buffer_rx_precent[sendID][2];
                            //校验位判断
                            char check_parity = 0x00;
                            for (int i = 3; i<3+n; i++)
                            {
                                check_parity = check_parity + buffer_rx_precent[sendID][i];
                            }

                            if (buffer_rx_precent[sendID][n+3] == check_parity)      //校验位
                            {   //校验位正确
                                memcpy(buffer_rx, buffer_rx_precent[sendID]+3, n);      //buffer_rx & n 待数据包解读
                                flag_complete = 1;

                                outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                                outfile << endl << "-- buffer_rx_precent[" << dec << (int)sendID <<"] check_parity Correct !! --" << endl;
                                outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                                for(int i=0; i<n_rx_precent[sendID]; i++)
                                {
                                    outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                                }
                                outfile << endl;
                                outfile.close();

                                //删除此包信息
                                if (n_rx_precent[sendID] - n -4 >0)
                                {
                                    memcpy(buffer_temp, buffer_rx_precent[sendID]+n+4, n_rx_precent[sendID] -n-4);
                                    memcpy(buffer_rx_precent[sendID], buffer_temp, n_rx_precent[sendID] -n-4);

                                    n_rx_precent[sendID] = n_rx_precent[sendID] -n-4;
                                }
                                else if (n_rx_precent[sendID] - n -4 == 0)
                                {
                                    n_rx_precent[sendID] = 0;
                                }
                                outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                                outfile  <<"After Move" << endl;
                                outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                                for(int i=0; i<n_rx_precent[sendID]; i++)
                                {
                                    outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                                }
                                outfile << endl;
                                outfile.close();

                            }
                            else
                            {   //校验位错误
                                outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                                outfile << endl << "-- buffer_rx_precent[" << dec << (int)sendID <<"] check_parity Error !! -- Error --" << endl;
                                outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                                for(int i=0; i<n_rx_precent[sendID]; i++)
                                {
                                    outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                                }
                                outfile << endl;
                                outfile.close();

                                //左移 2 位
                                memcpy(buffer_temp, buffer_rx_precent[sendID]+2, n_rx_precent[sendID] -2);
                                memcpy(buffer_rx_precent[sendID], buffer_temp, n_rx_precent[sendID] -2);
                                n_rx_precent[sendID] = n_rx_precent[sendID] -2;

                                outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                                outfile  <<"After Move" << endl;
                                outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                                for(int i=0; i<n_rx_precent[sendID]; i++)
                                {
                                    outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                                }
                                outfile << endl;
                                outfile.close();
                            }//End  if (buffer_rx_precent[sendID][n+3] == check_parity)      //校验位

                        }
                        else
                        {   // 包不完整
                            outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                            outfile << endl << "-- buffer_rx_precent[" << dec << (int)sendID <<"] >2 is not completed !! -- >2 -- Note --" << endl;
                            outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                            for(int i=0; i<n_rx_precent[sendID]; i++)
                            {
                                outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                            }
                            outfile << endl;
                            outfile.close();

                        } // End if (buffer_rx_precent[sendID][2] + 4 <= n_rx_precent[sendID])

                    }
                    else
                    {
                        //包的长度>32  错误! 人为限定了自定义包的大小
                        outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                        outfile << endl << "-- buffer_rx_precent[" << dec << (int)sendID <<"] Data_Bit >32 Error !! -- >32 -- Error --" << endl;
                        outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                        for(int i=0; i<n_rx_precent[sendID]; i++)
                        {
                            outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                        }
                        outfile << endl;
                        outfile.close();

                        //左移 2 位
                        memcpy(buffer_temp, buffer_rx_precent[sendID]+2, n_rx_precent[sendID] -2);
                        memcpy(buffer_rx_precent[sendID], buffer_temp, n_rx_precent[sendID] -2);
                        n_rx_precent[sendID] = n_rx_precent[sendID] -2;

                        outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                        outfile  <<"After Move" << endl;
                        outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                        for(int i=0; i<n_rx_precent[sendID]; i++)
                        {
                            outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                        }
                        outfile << endl;
                        outfile.close();
                    }

                }
                else if (n_rx_precent[sendID] > 0)
                {   // 包不完整
                    outfile.open("/home/ubuntu/serial_error/Buffer_rx_precent.txt",ios::app);
                    outfile << endl << "-- buffer_rx_precent[" << dec << (int)sendID <<"] <2 is not completed !! -- <2 -- Note --" << endl;
                    outfile << "buffer_rx_precent[" << dec << (int)sendID <<"]: ";
                    for(int i=0; i<n_rx_precent[sendID]; i++)
                    {
                        outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();
                }//End  else if (n_rx_precent[sendID] > 0)
                //----------------------------------------          模块数据 包处理&给出一个完整包  完成         --------------------------------------------------//

                //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 完整包信息解读+ROS发布           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
                if (flag_complete ==1)
                {   //如果存在一个完整包 则需要解读它
                    //根据自定义消息格式解析完整包
                    //buffer_rx[] & n 待解读的一个完整包

                    //判断是几号机接收的指令
                    if ( (buffer_rx[0]&0x0F) == CONFIG_ID )
                    {//如果接收的对象是本机

                        //待完成!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        //模块信息应答!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        //判断是否是应答信号部分!
                        if ( (n==2) && ( CONFIG_ID == ((buffer_rx[1]&0xF0) >> 4 )) )
                        {
                            outfile.open("/home/ubuntu/serial_error/Rx_Mission_Answer.txt",ios::app);
                            outfile << endl << "  ----  Received Handshake  !!  ----  !!" << endl;
                            outfile << " From  0x" << hex << sendID << ",   Order_num:"<< buffer_rx[2] <<endl;
                            outfile.close();
                        }
                        else
                        {
                            switch (sendID)
                            {
                            //侦察机
                            case 1:
                            case 2:


                                break;


                                //投掷机
                            case 3:
                            case 4:
                            case 5:
                            case 6:
                            case 7:
                            case 8:

                                /*投掷机 发送 给 地面站 的 指令格式
                                 *
                                 *char[0-2]
                                 * buffer[0] |char_8     [h7-l0]
                                 *   [h7-4  Drone_state	本机状态(/0b0000_正在执行任务	/0b0011_以上弹可以接收新目标	/0b1100_强制返航[判断是否需要无人机接手此任务])
                                 *   /h3-l0 CONFIG_ID	接收指令的地面站(0x0a)]	需要最先确认
                                 * buffer[1] |char_8     [h7-l0]
                                 *   [h7-4  保留ing
                                 *   /h3-l0 Order_num    指令编号	*用于握手应答判断   /本机tx缓存 此指令编号Num 即n_tx_current_flag
                                 * buffer[2] |char_8	[h7-l0]
                                 *   [h7-4  target_ID	投掷目标编号
                                 *   /3-2   target_type 投掷目标类型
                                 *   /1-l0  target_Num	已投掷次数
                                 * */
                                //根据指令格式 完成数据处理
                                /*serial -> ROS
                                 * int Drone_ID;       //投掷机 编号
                                 * int Accepted_ID;   //接收机 编号
                                 * int Drone_state;    //##本机状态	/111 - 0x0a接收到此指令后，即得知该机可以接收新的目标
                                 *                     //          /777    被强制返航   /0	本机有正在执行的任务
                                 * int target_ID;     // 投掷目标编号 1-10 根据被识别到的顺序
                                 * int target_type;     //投掷目标类型
                                 * int target_Num;    //对应投掷目标已经投掷次数	/无人机投掷时发布	/无人机被强制返航时发布
                                */
                                Drone_ID = sendID;
                                // buffer[0] |char_8     [h7-l0]
                                // [h7-4  Drone_state	本机状态(/0b0000_正在执行任务 ->ROS 0]	/0b0011_以上弹可以接收新目标 ->ROS 111]
                                //                               /0b1100_强制返航[判断是否需要无人机接手此任务])  -> ROS 777]
                                Drone_state = ((buffer_rx[0] & 0xF0) >> 4);
                                if (Drone_state == 0x00)
                                {
                                    Drone_state = 0;
                                }
                                else if (Drone_state == 0x03) //0b0000 0011
                                {
                                    Drone_state = 111;
                                }
                                else if (Drone_state == 0b00001100) //0b0000 1100
                                {
                                    Drone_state = 777;
                                }

                                //buffer[1] |char_8     [h7-l0]
                                //  [h7-4  保留ing
                                //  /h3-l0 Order_num    指令编号	*用于握手应答判断   /本机tx缓存 此指令编号Num 即n_tx_current_flag
                                Order_num = ((sendID << 4)|(buffer_rx[1] & 0x0F));

                                //* buffer[2] |char_8	[h7-l0]
                                //   [h7-4  target_ID	投掷目标编号
                                //   /h3-l0 target_Num	已投掷次数
                                target_ID = ((buffer_rx[2] & 0xF0) >> 4);
                                target_type = ((buffer_rx[2] & 0b00001100) >> 2);
                                target_Num = (buffer_rx[2] & 0b00000011);

                                //
                                //Received_target_data
                                Received_target_data.Drone_ID = Drone_ID;
                                Received_target_data.Drone_state = Drone_state;
                                Received_target_data.target_ID = target_ID;
                                Received_target_data.target_type = target_type;
                                Received_target_data.target_Num = target_Num;
                                Received_target_data.Accepted_ID = 0;

                                received_Throw_state_Pub.publish(Received_target_data);//解包后的数值发布到本机的话题当中去

                                outfile.open("/home/ubuntu/serial_error/Rx_Mission.txt",ios::app);
                                outfile << endl << " -- Updata Drone State! -- From 0x" << hex << Drone_ID << " -- "<< endl;
                                outfile << "Trans Info: "<<endl;
                                outfile << "Drone_state: " << dec << Drone_state << ", /0-performing Task /111-waiting New Task /777-Forced return"<<endl;
                                outfile << "target_ID: " << target_ID << endl;
                                outfile << "target_type: " << target_type << endl;
                                outfile << "target_Num: " << target_Num << endl;
                                outfile.close();

                                //待完成!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                //暂时 直接通信应答!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                Wbuffer_tx[0] = CustomData_Header_0;
                                Wbuffer_tx[1] = CustomData_Header_1;
                                Wbuffer_tx[2] = 0x02;            //数据位数
                                Wbuffer_tx[3+0] = sendID;       //应答数据0
                                Wbuffer_tx[3+1] = Order_num;    //应答数据1
                                Wbuffer_tx[3+2] = Wbuffer_tx[3+0] + Wbuffer_tx[3+1];      //校验位
                                Wn_tx = 6;
                                cout << endl << "Send_Data : ";
                                for(int i=0; i<Wn_tx; i++)
                                {   cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
                                }
                                cout << endl;
                                sp.write(Wbuffer_tx, Wn_tx);

                                break;


                            }


                        }

                    }


                }// End if (flag_complete ==1)

                //----------------------------------------          模块数据 完整包信息解读+ROS发布  完成         --------------------------------------------------//

            }   //End if (n_rx_precent[sendID]>0)

        }//End for (sendID = 1; sendID<11; sendID++)
        //----------------------------------------          模块数据 串口读取&发送&处理  完成         --------------------------------------------------//

        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 串口发送           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
        if (n_tx_current_flag > -1)
        {

            Wbuffer_tx[0] = CustomData_Header_0;
            Wbuffer_tx[1] = CustomData_Header_1;
            Wbuffer_tx[2] = (char) n_tx[n_tx_current_flag];            //数据位数
            memcpy(Wbuffer_tx+3, buffer_tx[n_tx_current_flag], n_tx[n_tx_current_flag]);

            char check_parity = 0x00;
            for (int i=0; i< n_tx[n_tx_current_flag]; i++)
            {
                check_parity = check_parity + buffer_tx[n_tx_current_flag][i];
            }
            Wbuffer_tx[n_tx[n_tx_current_flag]+3] = check_parity;      //校验位

            Wn_tx = n_tx[n_tx_current_flag] + 4;

            cout << endl << "Send_Data : ";
            for(int i=0; i<Wn_tx; i++)
            {   cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
            }
            cout << endl;

            sp.write(Wbuffer_tx, Wn_tx);

            outfile.open("/home/ubuntu/serial_error/Send_to_throw.txt",ios::app);
            outfile << endl << " -- Send_to_throw -- " << endl;

            outfile << endl << "Send_Data : ";
            for(int i=0; i<Wn_tx; i++)
            {   outfile << hex << (Wbuffer_tx[i] & 0xff) << " ";
            }
            outfile << endl;
            outfile.close();


            n_tx_current_flag = n_tx_current_flag-1;

        }//if (n_tx>0)
        //----------------------------------------          模块数据 串口发送  完成         --------------------------------------------------//

        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}

//##################################################            其他函数         ############################################################//
void show_connectedDevice()
{
    //显示已连接模块数量和编号
    cout << "already connected " << dec << connected_Count << " devices, Num: " ;//<< hex << (Connected_Num & 0xFFFF) << endl;
    int temp_t = 1;
    while (temp_t <15)
    {   //cout << "hex" << hex << (0x0001 << temp_t) << endl;
        if ( (Connected_Num & (0x0001 << temp_t)) == (0x0001 << temp_t))
        {
            cout << temp_t << ", ";
        }
        temp_t = temp_t + 1;
    }
    cout << endl;
}

