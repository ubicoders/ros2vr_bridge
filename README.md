# vrobots_mr

To run this, 

# 1. Install Python dependency

```
pip install ubicoders_vrobots
```

# 2. Build the ROS2 package

From your workspace, i.e robot_ws

```
colcon build --symlink-install
```

# 3. Launch

```
ros2 launch vrobots_mr vrobots_mr.launch.py
```

Observe the topics it publishes

```
ros2 topic echo /vr_mr_states 
```

# 4. Connect the Virtual Robot

Goto: https://www.ubicoders.com/virtualrobots/multirotor

Click "Connect" Button

# 5. Publish sample pwm

```
ros2 topic pub /vr_mr_pwm vrobots_msgs/VRobotActuator "pwm: [1500, 1500, 1500, 1510]"
```

# 6. Observe the simulation

From the web, click "Rest"

Does it move at least?

Does it fly properly?

# 7. Implement the controller

Define a new node and publish the data!

