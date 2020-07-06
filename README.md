# MuSHR-Control

Simple contollers for MuSHR. Code mainly modified from [CSE 478 20WI](https://gitlab.cs.washington.edu/cse478/20wi/ta_lab2).

More sophisticated Receding Horizon Control is in [mushr_rhc](https://github.com/prl-mushr/mushr_rhc).

Contact [Gilwoo Lee](mailto:gilwoo301@gmail.com) for questions regarding this repository.


## Testing the controllers with single car
Open 4 terminals, all sourced.

1. start the cars
```bash
roslaunch mushr_sim teleop.launch
```
2. start controller in a new terminal
```bash
roslaunch mushr_control mpc_controller.launch  car_name:=car
```

3. start rviz in a new terminal
```bash
rosrun rviz rviz -d config/car.rviz
```

5. start the runner script in a new terminal
```bash
roslaunch mushr_control runner_script.launch car_name:=car wait_for_signal:=true
```

6. Test one of the tracks by typing 1 ~ 5.


7. Publish messages to start path following
```bash
# In a terminal set with rosmaster 11311
rostopic pub /car/start_path_following std_msgs/Bool True
```
