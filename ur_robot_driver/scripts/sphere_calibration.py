#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from ctypes import create_unicode_buffer

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from getSensorInput import SensorGetter
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sphericalFit import SphericalFit

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class JointBasedMove():
    def go_to_joint_states(self, values, move_group):
        joint_goal = move_group.get_current_joint_values()

        for i in range(0, len(values)):
            joint_goal[i] = values[i]

        move_group.go(joint_goal, wait=True)
        move_group.stop()


class Calibration(object):

    def __init__(self):
        super(Calibration, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("sphere_calibration", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.sensorGetter = SensorGetter()

        self.sensorAttachementLength = 0.16
        self.safetyOffset = 0.01
        self.stepWidth = 0.0001

        self.jointMover = JointBasedMove()

    def planCartesianPath(self, x, y, z, oX, oY, oZ, oW):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.orientation.x = oX
        wpose.orientation.y = oY
        wpose.orientation.z = oZ
        wpose.orientation.w = oW
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        waypoints.append(copy.deepcopy(wpose))

        plan, fraction = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )
        return plan

    def executePlan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
    
    def getRadsFromDegrees(self, euler):
        result = []
        for i in range(0, len(euler)):
            result.append(self.getRadFromDeg(euler[i]))
        return result 

    def getRadFromDeg(self, value):
        return value*pi/180
    
    def getQuaternionForHorizontalMovement(self, vector):
        if(vector[0] != 0):
            euler = self.getRadsFromDegrees([0,90,0]) #front degrees are inverse to the robot panel. robot panel shows 270 but in python it is 360 - 270
        if(vector[1] > 0):
            euler = self.getRadsFromDegrees([270,0,0]) #right
        if(vector[1] < 0):
            euler = self.getRadsFromDegrees([90,0,0]) #left
        if(vector[2] != 0):
            euler = self.getRadsFromDegrees([180,0,0]) #top

        return quaternion_from_euler(euler[0], euler[1], euler[2])

    def getQuaternionForDiagonalMovement(self, vector):
        if(vector[0] == 0 and vector[1] == 0):
            euler = self.getRadsFromDegrees([180,0,0]) #top
        if(vector[0] > 0): #front
            euler = self.getRadsFromDegrees([0,135,0])
        if(vector[0] < 0): #back
            euler = self.getRadsFromDegrees([0,225,0])
        if(vector[1] > 0): #rigth
            euler = self.getRadsFromDegrees([225,0,0])
        if(vector[1] < 0): #left
            euler = self.getRadsFromDegrees([135,0,0])

        return quaternion_from_euler(euler[0], euler[1], euler[2])

    def MoveToSphere(self, args, vector, safetyOffset, sensorAttachementLength, stepWidth): #vector = zu fahrende richtung
        offset = safetyOffset + sensorAttachementLength
        currentX = float(args[1]) - offset * vector[0]
        currentY = float(args[2]) - offset * vector[1]
        currentZ = float(args[3]) - offset * vector[2]
        startX = currentX - offset * vector[0]
        startY = currentY - offset * vector[1]
        startZ = currentZ - offset * vector[2]
        #quaternion = self.getQuaternionForHorizontalMovement(vector)
        quaternion = self.getQuaternionForDiagonalMovement(vector)
        startPosPath = self.planCartesianPath(currentX, currentY, currentZ, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        self.executePlan(startPosPath)


        stopX = currentX + 2*safetyOffset * vector[0]
        stopY = currentY + 2*safetyOffset * vector[1]
        stopZ = currentZ + 2*safetyOffset * vector[2]

        sensorState = self.sensorGetter.getSensorState()
        
        while(sensorState and (not self.MaximumReached(currentX, stopX, vector[0])) and  (not self.MaximumReached(currentY, stopY, vector[1])) and (not self.MaximumReached(currentZ, stopZ, vector[2]))):
            currentX += vector[0] * stepWidth
            currentY += vector[1] * stepWidth
            currentZ += vector[2] * stepWidth
            plan = self.planCartesianPath(currentX, currentY, currentZ,  quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            self.executePlan(plan)
            sensorState = self.sensorGetter.getSensorState()

        result = self.move_group.get_current_pose().pose
        self.executePlan(self.planCartesianPath(startX, startY, startZ, quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        return result
    
    def MaximumReached(self, current, stop, vector):
        if(vector > 0):
            return current >= stop
        if(vector < 0):
            return current <= stop
        else: return False


    def HorizontalSphereApproach(self, args):
        result = []

        #forward
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-180, -90, -140, -130, 270, 0]), self.move_group)
        result.append(self.MoveToSphere(args, [1, 0, 0], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-180, -90, -140, -130, 270, 0]), self.move_group)
        #right
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-210, -130, -90, -140, 150, 0]), self.move_group)
        result.append(self.MoveToSphere(args, [0, 1, 0], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-210, -130, -90, -140, 150, 0]), self.move_group)
        #top
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-180, -90, -90, -90, 90, 0]), self.move_group)
        result.append(self.MoveToSphere(args, [0, 0, -1], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-180, -90, -90, -90, 90, 0]), self.move_group)
        #left
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-125, -130, -90, -140, 55, 0]), self.move_group)
        result.append(self.MoveToSphere(args, [0, -1, 0], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        self.jointMover.go_to_joint_states(self.getRadsFromDegrees([-125, -130, -90, -140, 55, 0]), self.move_group)

        print("Pose1:\n")
        print(result[0], "\n")
        print("Pose2:\n")
        print(result[1], "\n")
        print("Pose3:\n")
        print(result[2], "\n")
        print("Pose4:\n")
        print(result[3], "\n")

        return result

    def DiagonalSphereApproach(self, args):
        inverseSqrt2 = 1/sqrt(2)

        result = []
        rads = self.getRadsFromDegrees([-180, -90, -90, -90, 90, 0])

        #top
        self.jointMover.go_to_joint_states(rads, self.move_group)
        result.append(self.MoveToSphere(args, [0, 0, -1], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        self.jointMover.go_to_joint_states(rads, self.move_group)
        #right
        #result.append(self.MoveToSphere(args, [0, inverseSqrt2, -inverseSqrt2], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        #self.jointMover.go_to_joint_states(rads, self.move_group)
        #left
        #result.append(self.MoveToSphere(args, [0, -inverseSqrt2, -inverseSqrt2], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        #self.jointMover.go_to_joint_states(rads, self.move_group)
        #front
        #result.append(self.MoveToSphere(args, [inverseSqrt2, 0, -inverseSqrt2], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        #self.jointMover.go_to_joint_states(rads, self.move_group)
        #back
        #result.append(self.MoveToSphere(args, [-inverseSqrt2, 0, -inverseSqrt2], self.safetyOffset, self.sensorAttachementLength, self.stepWidth))
        #self.jointMover.go_to_joint_states(rads, self.move_group)

        return result

    def ConvertPosesToPoints(self, poses):
        result = []
        for i in range(0, len(poses)):
            tupel = (poses[i].position.x, poses[i].position.y, poses[i].position.z) 
            result.append(tupel)
        return result

        
def main():
    try:
        args = sys.argv
        calibration = Calibration()
        sphericalFit = SphericalFit()
        result = calibration.DiagonalSphereApproach(args)
        radius, x0, y0, z0 = sphericalFit.doSphereFit(calibration.ConvertPosesToPoints(result))
        print('Radius: ', radius, 'Center: (', str(x0), '|', str(y0), '|', str(z0), ')')
        print(result)
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()