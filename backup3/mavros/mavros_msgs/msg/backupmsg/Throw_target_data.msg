#Throw_target_data
#投掷组收到的要执行的指令

# 投掷机编号
int32 Drone_ID

#返航标志	0-不返航	1-返航
int32 flag_Return

#投掷目标相关
int32 target_type		#投掷目标类型 1/2/3
int32 target_Num		#投掷目标编号 1-10 根据被识别到的顺序

float64 latitude		#投掷点GPS 纬度
float64 longitude		#投掷点GPS 经度

float32 enterangle		#目标3-本机攻击进入角
int32 partner1			#目标3-协作无人机1
int32 partner2			#目标3-协作无人机2
