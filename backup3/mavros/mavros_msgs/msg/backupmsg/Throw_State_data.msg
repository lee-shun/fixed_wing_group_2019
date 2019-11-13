#Throw_State_data.msg
#投掷组收到的要发送的状态

# 投掷机编号
int32 Drone_ID

#接收机 编号	0x01-0x0a
int32 Accepted_Num

#本机状态	
#	111 - 0x01接收到此指令后，即得知该机可以接收新的目标
#	777	被强制返航
#	0	表示本机有正在执行的任务
int32 Drone_state

#投掷目标编号 1-10 根据被识别到的顺序
int32 target_Num

#对应投掷目标已经投掷次数	/无人机投掷时发布	/无人机被强制返航时发布
int32 target_type
