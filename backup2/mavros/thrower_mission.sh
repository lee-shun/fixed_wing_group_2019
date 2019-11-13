gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch mavros thrower_mission.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros thrower_serial.launch; exec bash"' \
--window -e 'bash -c "sleep 8; rostopic echo /Mission_Data_Throw_target; exec bash"' \
--window -e 'bash -c "sleep 8; rostopic echo /Waitsend_Data_Drone_State; exec bash"' \

#--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch; exec bash"' \
#--tab -e 'bash -c "sleep 3; roslaunch dji_sdk sdk.launch; exec bash"' \
#--tab -e 'bash -c "sleep 6; cd /home/ubuntu/ws_drop; ./Sh_MissionDrop.sh; exec bash"' \

#	投掷机 任务执行程序
#	现有功能	完成1/2目标投掷	返航	沙包装载	




