#Control Flag

int32 drone_id
int32 flag_control
int32 comid

#Control Command
int32 flag_body
float32[3] pos_drone_sp
float32 yaw_drone_sp #du

#Mission Command
#command = 0 默认值
#command = 1 起飞 开始执行任务
#command = 2 统一返航 已经完成3个目标投掷 任务结束
#command = 3 紧急情况 统一原地降落

int32 mission_command

