#
# Move arm or adjust gripper
#

byte MOVE_ARM=0
byte MOVE_GRIPPER=1

byte type                   # move the arm or the gripper?

geometry_msgs/Pose goal     # goal for arm
float64 command    # width to open gripper

duration move_time
