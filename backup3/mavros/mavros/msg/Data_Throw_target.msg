# Data_Throw_target.msg
## 投掷组收到的要执行的指令
## 侦察机发送给地面站的信息

int32 Accepted_ID	#接收机 编号	0x01-0x0a
int32 Drone_ID		#本机 编号

int32 flag_Return	#返航标志	/0-不返航	/1-返航

#投掷目标相关
int32 target_ID		#投掷圆编号 按被识别到的顺序	/投掷机以地面站基准	/侦察机以其本机为基准
int32 target_type	#投掷圆内数字	1/2/3
int32 target_Num	#已投掷次数	/投掷机投掷时更新	/投掷机被强制返航时更新

float64 latitude	#投掷圆心GPS 纬度
float64 longitude	#投掷圆心GPS 经度

float32 enterangle	#目标3-本机攻击进入角
int32 partner1		#目标3-协作无人机1
int32 partner2		#目标3-协作无人机2
