# Command from leader to follower
Header header

# Mode: 0=IDLE, 1=TELEOP, 2=FORMATION, 3=FOLLOW
uint8 mode
uint8 MODE_IDLE=0
uint8 MODE_TELEOP=1
uint8 MODE_FORMATION=2
uint8 MODE_FOLLOW=3

# Formation type (valid when mode==FORMATION)
uint8 formation
uint8 FORMATION_COLUMN=0
uint8 FORMATION_LINE=1
uint8 FORMATION_TRIANGLE_LEFT=2
uint8 FORMATION_TRIANGLE_RIGHT=3

# Desired follower offset in leader body frame
float64 offset_x
float64 offset_y
float64 offset_yaw

# Leader velocity (for dead reckoning if follower loses comms briefly)
float64 leader_vx
float64 leader_vyaw

# Pre-computed target pose for follower (in leader odom frame)
geometry_msgs/Pose target_pose

# Speed limit for follower
float64 speed_limit
