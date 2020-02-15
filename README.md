# Model Predictive Control Local Planner Plugin for move_base navigation stack
Works in ros-melodic version.

Temporary working environment (morse simulator and maps from my master course).

# Before Run
Install Ipopt: Please refer the tutorial in "document/ipopt_install".

Put this in your launch file like a normal move_base plugin and load the params file:
  
```bash
  <param name="base_local_planner" value="mpc_local_planner_mb/MPCLocalPlannerMBROS" />
  <rosparam file="$(find mpc_local_planner_mb)/cfg/mpc_params.yaml" command="load" />
```

# Parameters yaml file:

* debug_info: For internal ros warnings info for computations of steering, speed, cte and more values.
* delay_mode: Added a delay mode equals to differential time(dt)
* max_speed: Max speed valye
* min_speed: Min speed value
* goal_radius: value for goal radius (in meters)
* controller_freq: Value to syschronize the planner with control loop of move base, (in our case didn't solve the problem of control loop failure)
* mpc_max_steering: Value of the maximum available steering
* mpc_max_throttle: Value for the maximum accelaration
* mpc_steps: Value for setting the horizon of MPC ( really big means high computation)

* The rest values were based in the external reference



# Coming
New rviz.

New map file.

Code optimization.

Yaw tolerance.

Global functionality.


# External Reference
HyphaROS MPC MiniCar(https://hypharosworkshop.wordpress.com/)
