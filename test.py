import dvrk
import PyKDL
import math
import numpy
import rospy
# Create a Python proxy for PSM1, name must match ros namespace
mtml = dvrk.mtm('MTML')

# # You can home from Python
mtml.enable()
mtml.home()

# # retrieve current info (numpy.array)
# p.measured_jp()
# p.measured_jv()
# p.measured_jf()

# # retrieve PID desired position and effort computed
# p.setpoint_jp()
# p.setpoint_jf()

# # retrieve cartesian current and desired positions
# # PyKDL.Frame
# p.measured_cp()
# p.setpoint_cp()

# # move in joint space
# # move is absolute (SI units)

# # move multiple joints
# p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))


# # start position
# goal = p.setpoint_cp()
# # move 5cm in z direction
# goal.p[2] += 0.05
# p.move_cp(goal).wait()

# # start position
# goal = p.setpoint_cp()
# # rotate tool tip frame by 25 degrees
# goal.M.DoRotX(math.pi * 0.25)
# p.move_cp(goal).wait()

#move jaw
print(mtml.gripper.measured_js())
# rospy.sleep(0.01)