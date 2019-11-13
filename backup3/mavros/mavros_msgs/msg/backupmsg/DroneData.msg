#Drone State
int32 comid
int32 Num_statemachine

float32[3] pos_drone
float32 vel_drone_xy
float32 vel_drone_z
float32 yaw_drone                # du

#Control State
#int32 flag_body # 0 for enu 1 for body
#float32[3] pos_drone_sp #depends on the flag_body
#float32 yaw_drone_sp

#float32[4] control_cmd

#Vision State
int32 flag_vision
float32[3] pos_target
int32 flag_drop

