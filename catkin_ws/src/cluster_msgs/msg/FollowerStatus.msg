# Status feedback from follower to leader
Header header

# State: 0=IDLE, 1=TRACKING, 2=LOST, 3=EMERGENCY
uint8 state
uint8 STATE_IDLE=0
uint8 STATE_TRACKING=1
uint8 STATE_LOST=2
uint8 STATE_EMERGENCY=3

# Tracking errors
float64 error_x         # cross-track error (m)
float64 error_y         # along-track error (m)
float64 error_yaw       # heading error (rad)
float64 error_dist      # Euclidean distance error (m)

# Whether leader odometry is being received
bool leader_visible
