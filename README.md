# ssm_safety_ros

A ROS2 implementation of the ssm_safety library to slow robots down in proximity of humans.

## Example

1. Launch your robot
2. Launch node
```
ros2 launch ssm_safety_ros test.launch.py
```
3. Publish a ```geomtry_msgs::PoseArray``` msg on topic ```/poses```, e.g.:
```
ros2 topic pub -r 10 /poses geometry_msgs/PoseArray "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
poses:
- position:
    x: 1.5
    y: 0.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
4. ```ros2 topic echo /speed_ovr```
5. Move the robot around. When it approaches the msg's pose, the value of ```/speed_ovr``` will decrease.

**IMPORTANT:**To actually slow the robot down, you need a controller that takes the /speed_ovr value and scale the robot'speed accordingly.
An example of such controller can be found [here](https://github.com/JRL-CARI-CNR-UNIBS/scaled_follow_joint_trajectory_controller_ros2).

## Options

See ```config/example_params.yaml```

## References

The concept behind the implementation of package velocity_scaling_iso15066 was described in Sec. III-F of the following [paper](https://arxiv.org/pdf/2210.11655.pdf ):
```
@article{faroni2022safety,
  title={Safety-aware time-optimal motion planning with uncertain human state estimation},
  author={Faroni, Marco and Beschi, Manuel and Pedrocchi, Nicola},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={12219--12226},
  year={2022},
  publisher={IEEE}
}
```

