# Place for experiments and a work in progress.
# Author: Michal Bednarek

# Force mode example
import robot_controller


# communicate with a robot
manipulator = robot_controller.Ur3("150.254.47.160", 30003, 30002)

force_traj = list()
pose = manipulator.get_pose()
pose[2] -= 0.1

for i in range(3):
    pose[1] += 0.05
    pt = manipulator.create_move_command(pose, is_movej=False, a=0.2, v=0.2)
    force_traj.append(pt)

print manipulator.execute_in_force_mode(force_traj)
