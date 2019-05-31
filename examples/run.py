# Example of use.
# Author: Michal Bednarek

from src.manipulators import Ur3

# communicate with a robot
manipulator = Ur3("150.254.47.150", 30003, 30002)

# create trajectory as a list of poses X, Y, Z, A, B, C
# trajectory = list()
# pose1 = manipulator.get_pose()
# pose2 = pose1
# pose2[0] -= 0.1
# pose2[2] += 0.1
# pose3 = pose2
# pose3[1] += 0.1
# trajectory.append(pose2)
# trajectory.append(pose3)

# move robot
# manipulator.move(trajectory, False, a=0.1, v=0.5)
