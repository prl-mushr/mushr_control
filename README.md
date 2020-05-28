# MuSHR-Control

Simple contollers for MuSHR. Code mainly modified from [CSE 478 20WI](https://gitlab.cs.washington.edu/cse478/20wi/mushr_control).

More sophisticated Receding Horizon Control is in [mushr_rhc](https://github.com/prl-mushr/mushr_rhc).

Contact [Gilwoo Lee](mailto:gilwoo301@gmail.com) for questions regarding this repository.


# Testing the controllers with multi-car

1. Start the map server
```
roslaunch mushr_control map_server.launch
```

2. start the cars
```
roslaunch mushr_sim multi_teleop.launch
```

3. start controllers for both cars
```
bash scripts/multi_car_mpc.sh
```

4. In rviz, move the cars to different locations

5. start the runner script 
```
roslaunch mushr_control runner_script.launch car_name:=car2
```

6. Test one of the tracks.
