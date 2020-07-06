
## Testing the controllers with multi-car
Open 7 terminals, all sourced.

1. start the cars 
```bash
roscd mushr_sim
cd scripts
./start_single_car.sh 11311 car1 ["'/car1/start_path_following'", "'/car2/car_pose'","'/car3/car_pose'"]
# In a new terminal
./start_single_car.sh 11312 car2 ["'/car2/start_path_following'", "'/car1/car_pose'","'/car3/car_pose'"]
```

2. Start the base station (with default port 11315)
```bash
# In the same scripts directory
./start_basestation.sh
```

3. start controllers and runner script for both cars.
```bash
# In the first terminal,  (or a terminal set with rosmaster 11311 and ROS_IP)
roslaunch mushr_control mpc_controller.launch  car_name:=car1 &
# In the second terminal, (or a terminal set with rosmaster 11312)
roslaunch mushr_control mpc_controller.launch  car_name:=car2 &
```

4. Start rviz from the basestation rosmaster with multicar.rviz. 
```bash
export ROS_IP=$(ifconfig enp5s0 | awk /inet\ /'{print $2}')
export ROS_MASTER_URI=http://$ROS_IP:11315
rosrun rviz rviz -d config/multicar.rviz
```
In rviz, move the cars to different locations.


5. start the runner script in two terminals
```bash
# In the first terminal,  (or a terminal set with rosmaster 11311)
roslaunch mushr_control runner_script.launch car_name:=car1 wait_for_signal:=true 
# In the second terminal, (or a terminal set with rosmaster 11312)
roslaunch mushr_control runner_script.launch car_name:=car2 wait_for_signal:=true 
```

6. Test one of the tracks by typing 1 ~ 5.


7. Publish messages to start path following. The signal can be sent from the base station (just for the sake of convenience.)
```bash
# In a terminal set with rosmaster 11315
rostopic pub /car1/start_path_following std_msgs/Bool True 
# In a terminal set with rosmaster 11315
rostopic pub /car2/start_path_following std_msgs/Bool True
```
