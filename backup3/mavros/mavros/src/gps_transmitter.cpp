/**
    2019/04/28
    非常重要的数据
    用于订阅
  */
//将本机的gps消息转化为要发送的格式，通过串口发送出去


#include <ros/ros.h>

//mavros相关头文件
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>




#include <serial/serial.h>
#include "std_msgs/String.h"
#include <sensor_msgs/NavSatFix.h>

using namespace std;
sensor_msgs::NavSatFix received_gps;
sensor_msgs::NavSatFix need_to_send_gps;
int cot=1;

using namespace std;

//##################################################            全局变量         ############################################################//
//##################################################            全局变量         ############################################################//

//----------------------------------------          无线模块初始化相关          ----------------------------------------//
//通信模块配置 相关帧头
//上电节点配置  本机的ID
const uint8_t CONFIG_Header_0 = 0xEB;               //帧头0
const uint8_t CONFIG_Header_1 = 0x90;               //帧头1
const uint8_t CONFIG_ID =       0x01;               //通信模块编号 | 与飞机编号相同母鸡为1号  

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

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    received_gps = *msg;
    cout <<cot<<" "<<"the transfered gps is "<<endl << received_gps.latitude << endl;
    cout <<cot<<" "<<"the transfered gps is "<<endl << received_gps.longitude << endl;
    cot=cot+1;
}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "GPS_transmitter");

    ros::NodeHandle nh;


    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, gps_callback);

    //trans_pub此处的命名是为了后面发布的
    //advertise
    //advertise<消息定义>("话题的名称",队列大小)
    //  ros::Publisher trans_pub = nh.advertise<sensor_msgs::NavSatFix>("transfered_gps", 100);

    ros::Rate loop_rate(20);


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
    //  ----------------------------------------          串口打开 完成         --------------------------------------------------//

   //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块 初始配置           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    // 串口打开后，配置模块 地址 EB 90 + 飞机编号
    Wbuffer_tx[0] = CONFIG_Header_0;     //配置帧头 0xEB
    Wbuffer_tx[1] = CONFIG_Header_1;     //配置帧头 0x90
    Wbuffer_tx[2] = CONFIG_ID;           //本机编号 0x01即为母鸡站


                            cout<<"Send_Configuration_Set: ";
                            for(int i=0; i<3; i++)
                            {
                                cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
                            }
                            cout << endl << "Please Wait!!" << endl << endl;


    int flag_config = 1;//强制进入
    while(flag_config&ros::ok())
    {
        ros::spinOnce();

        sp.write(Wbuffer_tx, 3);//配置模块 地址 EB 90 + 飞机编号

        usleep(25000); 

        Rn = sp.available();  //获取缓冲区内的字节数
        cout<<"rn===aaa=="<<Rn<<endl;

                                            //         const uint8_t CONFIG_Header_0 = 0xEB;               //帧头0
                                            // const uint8_t CONFIG_Header_1 = 0x90;               //帧头1
                                            // const uint8_t CONFIG_ID =       0x03;               //通信模块编号 | 与飞机编号相同   0x01&0x02_侦察机    0x03-0x08_投掷机    0x0a_地面站

                                            // //接收数据的模块的针头
                                            // const uint8_t Data_HEADER_0	= 0X9F;                 //帧头0
                                            // const uint8_t Data_HEADER_1	= 0XE4;                 //帧头1

                                            // //准备发送的数据帧头--自定义的，解包，分割用
                                            // const uint8_t CustomData_Header_0	= 0X31;         //帧头0
                                            // const uint8_t CustomData_Header_1	= 0X6B;         //帧头1
        if(Rn!=0)
        {
            Rn = sp.read(Rbuffer_rx, Rn);  //读出数据// 本机配置消息的读取 格式： EB 90 +配置状态（0为成功）+ 本机编号CONFIG_ID
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








    while (ros::ok())
    {

        ros::spinOnce();//接受本机的ros消息


        double lat;
        double lon;
        int data_lat[5],data_lon[5];
        int i=0;

        lat=received_gps.latitude;
        lon=received_gps.longitude;
        cout << "lat= "<<  lat <<endl;
        cout << "lon= "<<  lon <<endl;


        // lat = 39.7285473000;
        // lon = 116.1636696;

        //#########################################解算要发送的数据，转换成单位格式###################################################

        //纬度
        for(i=0;i<5;i++){

            data_lat[i] = (int)lat;
            lat = (lat-data_lat[i])*100;
        }

        for(i=0;i<5;i++){

            cout<< "data_lat"<<"["<<i<<"]="<<dec<< data_lat[i] <<";"<<endl;

        }

        //经度

        for(i=0;i<5;i++){
            //        cout<< "lon=" << lon <<endl;
            data_lon[i] = (int)lon;
            lon = (lon-data_lon[i])*100;
        }
        for(i=0;i<5;i++){
            cout<< "data_lon"<<"["<<i<<"]="<<dec<< data_lon[i] <<";"<<endl;
        }
        //#############################################################################################


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 串口发送           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
       
        int   Wn_tx ;//用来打印的计数器
        int sum=0;

        Wbuffer_tx[0] = CustomData_Header_0;
        Wbuffer_tx[1] = CustomData_Header_1;
        Wbuffer_tx[2] = 0x0a ;            //数据位数,经纬度的信息是两个5位加一个校验为



        //依次存入纬度和经度
        for(i=0;i<5;i++){

            Wbuffer_tx[3+i] = data_lat[i];
            sum=sum+data_lat[i];
            cout<< "hdfshfdhflon"<<"["<<i<<"]="<<dec<< (Wbuffer_tx[3+i]&0xff)<<";"<<endl;
        }



        for(i=0;i<5;i++){

            Wbuffer_tx[8+i] = data_lon[i];
            sum=sum+data_lon[i];
        }



        Wbuffer_tx[12] = sum;
        Wn_tx = 13;//用来打印的计数器

        cout << endl << "Send_Data : ";

        for(int i=0; i<Wn_tx; i++){
            //    cout << Wbuffer_tx[i] << " ";
            cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
        }
        cout << endl;

        sp.write(Wbuffer_tx, Wn_tx);//发送数据
        loop_rate.sleep();

        //if (n_tx>0)
        //----------------------------------------          模块数据 串口发送  完成         --------------------------------------------------//











    }





//     //关闭串口s
   sp.close();
    return 0;
}
