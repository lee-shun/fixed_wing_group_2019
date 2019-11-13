# Data_Drone_State.msg
#投掷组收到的要发送的状态

int32 Accepted_ID	#接收机 编号	0x01-0x0a

int32 Drone_ID		#本机 编号

int32 Drone_state	#本机状态
#投掷机	/111-可接收新目标	/777-被强制返航	/0-正在执行的任务
#侦察机	/111-等待开始		/777-被强制返航	/0-正在执行的任务

int32 target_ID		#投掷圆编号 /根据被识别到的顺序(地面站基准)
int32 target_type	#投掷圆内数字
int32 target_Num	#已投掷次数	/投掷机投掷时更新	/投掷机被强制返航时更新

