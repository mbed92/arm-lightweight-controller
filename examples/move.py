
# Example of use. Execute some trajectory.
# Author: Michal Bednarek

import robot_controller

# communicate with a robot
manipulator = robot_controller.Ur3("192.168.1.211", 30003, 30002)
#print manipulator.get_pose()

# create trajectory as a list of poses X, Y, Z, A, B, C
trajectory = list()
pose1 = manipulator.get_pose()
pose2 = list(pose1)
pose2[0] -= 0.2
pose2[2] += 0.2
pose3 = list(pose2)
pose3[0] += 0.2
pose3[2] -= 0.2
trajectory.append(pose2)
trajectory.append(pose3)

print pose2
print pose3

# move robot
manipulator.move(trajectory, False, a=0.1, v=0.5)
