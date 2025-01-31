# Initialize joint states
ros2 topic pub /joint_state sensor_msgs/msg/JointState "{
  header: {
    stamp: {sec: 0}, 
    frame_id: 'base_link'
  },
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"

# Publish target pose
ros2 topic pub -r 100 /target_pose geometry_msgs/msg/Pose "{
  position: {
    x: 0.2,
    y: 0.0,
    z: 0.3
  },
  orientation: {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0
  }
}"

# Set target joint states
ros2 topic pub -r 100 /target_joint_state sensor_msgs/msg/JointState "{
  header: {
    stamp: {sec: 0},
    frame_id: 'base_link'
  },
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
  velocity: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"

# Monitor topics
ros2 topic echo /target_joint_state
ros2 topic echo /joint_state

# CAN bus setup
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Run tests
colcon test --packages-select kinematics --event-handlers console_direct+