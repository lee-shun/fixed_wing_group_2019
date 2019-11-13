gnome-terminal --window -e 'bash -c "roslaunch mavros thrower_serial.launch; exec bash"' \
--window -e 'bash -c "sleep 5; rostopic echo /Mission_Data_Throw_target; exec bash"' \
--window -e 'bash -c "sleep 5; rostopic echo /Waitsend_Data_Drone_State; exec bash"' \

##--tab -e 'bash -c "sleep 3; rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0; exec bash"' \
##--tab -e 'bash -c "sleep 10; roslaunch dji_sdk_demo camera_gimbal.launch; exec bash"' \



#	投掷机 通信模块 测试
# 测试其是否能正确 收发串口信息&收发ROS msg





