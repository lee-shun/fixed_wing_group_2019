/* 串口通信 程序 银河
 * 原作者：zzy 
 * 作者:  SSS & LS
 * 日期:    2019.04.29
 * 功能:
 *  1. 接受来自某一个通信模块的数据进行处理
    2. 将解包后的数据发布到本机的ros话题当中       
 *  3. 订阅自身的ros话题，通过串口传送出去        
 *                        
 */
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


#include <sensor_msgs/NavSatFix.h>

using namespace std;
//##################################################            全局变量         ############################################################//

//----------------------------------------          无线模块初始化相关          ----------------------------------------//
//通信模块配置 相关帧头
//上电节点配置  本机的ID
const uint8_t CONFIG_Header_0 = 0xEB;               //帧头0
const uint8_t CONFIG_Header_1 = 0x90;               //帧头1
const uint8_t CONFIG_ID =       0x0a;               //通信模块编号 | 与飞机编号相同   0x01&0x02_侦察机    0x03-0x08_投掷机    0x0a_地面站

//接收数据的模块的针头
const uint8_t Data_HEADER_0	= 0X9F;                 //帧头0
const uint8_t Data_HEADER_1	= 0XE4;                 //帧头1

//准备发送的数据帧头--自定义的，解包，分割用
const uint8_t CustomData_Header_0	= 0X31;         //帧头0
const uint8_t CustomData_Header_1	= 0X6B;         //帧头1



//----------------------------------------          无线模块包处理相关          ----------------------------------------//

//统计连接在内的模块
char16_t Connected_Num = 0x0001 << CONFIG_ID;       //统计连接在系统内的模块（暂时未用到）
int connected_Count = 0;                            //统计连接的模块计数（暂时未用到）

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



int complate_good_flag=0;
 int check_good_flag=0;


//##################################################            回调函数         ############################################################//
// void WaitSend_State_data_callback(const dji_sdk::Send_State_data::ConstPtr& msg)
// {

// }

//##################################################            主函数         ##############################################
int main(int argc, char **argv)
{


    ros::init(argc, argv, "GPS_receiver");//节点的名字

    ros::NodeHandle nh;

   //需要通过串口发送的指令  先订阅本机的消息，拿进来
    // ros::Subscriber Send_State_data_Sub = nh.subscribe<dji_sdk::Send_State_data>("/WaitSend_State_data", 10, WaitSend_State_data_callback);


    //接收到之后需要执行的指令  --通过ROS发布给控制飞机的节点
    ros::Publisher trans_pub = nh.advertise<sensor_msgs::NavSatFix>("received_leader_gps", 100);
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
    //cout<<  "666! CONFIG_ID="<<(CONFIG_ID& 0xff)<<endl;
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
   //----------------------------------------          模块 初始配置 完成         --------------------------------------------------//         模块 初始配置 完成         --------------------------------------------------//



     //>>>>>//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 串口读取&发送&处理           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
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

            //show_connectedDevice();     //显示已连接模块数量和编号

        }//if(Rn!=0)
        else
        {
            sendID = 0x00;  //记录已连接模块ID清零
            cout << endl << endl << "-------------this is the new data-----------" << endl << "Serial Received NO Data & No device connected !" <<endl;
        }


    //   //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据包处理&给出一个完整包已经存到了buffer_rx_precent[sendID]    <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
        
    //     // for (sendID = 1; sendID<11; sendID++)
    //     sendID = 0x01; 
        

    //    //############################################################################### ########################################
    //     //缓存里有数据，进行检验处理，结果在int check_good_flag;int complate_good_flag=里存放                    
    //     if (n_rx_precent[sendID]>0)
    //     {
    //             //要那母鸡的数据了
    //             //找出包
    //             //找到开头帧 316B
    //             n_present = 0;

    //             flag_find = 0;
    //              //##############################################################################
    //             while (flag_find ==0)
    //             {
    //                 if (n_present+1 == n_rx_precent[sendID])
    //                 {
    //                     flag_find = 1;
    //                     n_present = n_rx_precent[sendID];
    //                     //没有找到帧头 到最后了
    //                 }
    //                 else if (buffer_rx_precent[sendID][n_present+0]==CustomData_Header_0 && buffer_rx_precent[sendID][n_present+1]==CustomData_Header_1)
    //                 {
    //                     flag_find = 1;
    //                 }
    //                 else
    //                 {
    //                     flag_find = 0;
    //                     n_present = n_present+1;
    //                 }
    //             }
    //              //##############################################################################
    //             //如果这里拿进来缓存的数据的开头不是316b，说明有杂鱼混进来了，即如果n_present不等于0就说明凉了
              
    //             if (n_present != 0)
    //             {
                    
    //                 cout <<"这个的开头不是316b,有杂鱼混进来了qaq"<<endl;
    //                 //前移到[0]
    //                 if (n_rx_precent[sendID] - n_present >0)
    //                 {
    //                     memcpy(buffer_temp, buffer_rx_precent[sendID]+n_present, n_rx_precent[sendID] - n_present);
    //                     memcpy(buffer_rx_precent[sendID], buffer_temp, n_rx_precent[sendID] - n_present);

    //                     n_rx_precent[sendID] = n_rx_precent[sendID] - n_present;
    //                     n_present = 0;
    //                 }
    //                 else if (n_rx_precent[sendID] - n_present == 0)
    //                 {
    //                     n_rx_precent[sendID] = 0;
    //                     n_present = 0;
    //                 }
    //             }
                 
    //              //##############################################################################
    //             //到这里应该所有的都对齐了，头都是316b
                
    //                    //判断是否是一个完整的包

    //                    int n;
                       
    //                     if (buffer_rx_precent[sendID][2] + 4 <= n_rx_precent[sendID])
    //                     {  
                             
    //                          complate_good_flag=1;//是一个完整的包


    //                          int  check_parity;
    //                          n = buffer_rx_precent[sendID][2];

    //                            for (int i = 3; i<3+n; i++)
    //                         {
    //                             check_parity = check_parity + buffer_rx_precent[sendID][i];
    //                         }

                           
    //                        if (buffer_rx_precent[sendID][n+3] == check_parity)
    //                        {
    //                             check_good_flag=1;
    //                             cout<<"校验位通过，撒花～～～～～～"<<endl;
    //                        }
    //                        else
    //                        {
    //                             check_good_flag=0;
    //                             cout<<"校验位没通过，就差一点了"<<endl;
    //                        }

    //                     }//判断是否是一个完整的包

    //                     else
    //                     {
    //                         int complate_good_flag=0;
    //                         cout<<"缓存的数组里的不是一个完整的包没，下一个"<<endl;
    //                     }




        
        
    //     }

    //      if(check_good_flag&&complate_good_flag)
    //     {
         

    //             //--------------------------------------------------------------------------------------
    //             double lat;
    //             double lon;
    //             int data_lat[5],data_lon[5];
    //             int i;
    //             //读取纬度
    //             for(i=0;i<5;i++){

    //                 data_lat[i] = buffer_rx_precent[sendID][n_last+3+i];

    //             }
    //             for(i=0;i<5;i++){

    //                 cout<< "data_lat"<<"["<<i<<"]="<<dec<< data_lat[i] <<";"<<endl;

    //             }


    //             //读取经度

    //             for(i=0;i<5;i++){
    //                 data_lon[i] = buffer_rx_precent[sendID][n_last+8+i];

    //             }
    //             for(i=0;i<5;i++){
    //                 cout<< "data_lon"<<"["<<i<<"]="<<dec<< data_lon[i] <<";"<<endl;
    //             }
    //             //-------------------------------------------------------------------------------------


    //     }



     }//while(ros::ok())




//关闭串口
    sp.close();




return 0;
}




