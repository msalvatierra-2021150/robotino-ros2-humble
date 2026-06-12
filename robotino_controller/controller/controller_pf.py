import rclpy

import numpy as np
import math


from rclpy.node import Node

# geometry_msgs.msg.Twist is the type of message that this code sends 
# (Linear and angular velocity) fro the 2D Robots

from geometry_msgs.msg import Twist

# The sensor information is transferred as an ofometry message
# It contains info, about position orientation and velocities 
from nav_msgs.msg import Odometry

# The Lidar sensor information
# The messages are received throught the topic /lidar_scan

from sensor_msgs.msg import LaserScan

# We use this function to convert the orientation expressed as a quartenion into an angle
# THe orientation in the odometry message is expressed as a quartenion but we convert it to angle
from tf_transformations import euler_from_quaternion

# To track time
import time

# Topics
# This topic is used to send TWist control commands
topic1 = '/cmd_vel'

# This topic is used to receive Pose sensor measurements
topic2 = '/odom'

# This topic is used to receive LiDAR scans
topic3 = '/lidar_scan'

class ControllerNode(Node):

    def __init__(self, xdu, ydu, kau, kru, kthetau, gstaru, eps_orientu, eps_controlu):
        # call the constructor of the base (parent) class
        # and give the name of the node
        super().__init__('controller_node')

        # (xdp, ydp) - desired point
        self.xdp = xdu
        self.ydp = ydu

        # control parameter for the attraction force
        self.kap = kau

        # control parameter for the repulsive force
        self.krp = kru

        # ktheta - control parameter for the orientation controller
        self.kthetap = kthetau

        # gstar parameter for limiting the influence of the repulsive function
        self.gstarp = gstaru

        # tolerance for performing first orientation adjustment and then control
        self.eps_orient = eps_orientu

        # tolerance for turning off the controller
        # if we approximately reached the desired position
        self.eps_control = eps_controlu

        # this is the pose that is continuously being updated
        # on the basis of the message received from the robot
        self.OdometryMsg = Odometry()

        # this is the lidar scan message that is continuously being updated
        self.LidarMsg = LaserScan()

        # measure the initial time
        self.initialTime = time.time()

        # this is the time stamp when the Odometry message is received
        self.msgOdometryTime = time.time()

        # this is the time stamp when the Lidar message is received
        self.msgLidarTime = time.time()

        # this is the control message we are sending
        self.controlVel = Twist()

        # set the control velocity to zero at the beginning

        # components of the linear velocity vector
        self.controlVel.linear.x = 0.0
        self.controlVel.linear.y = 0.0
        self.controlVel.linear.z = 0.0

        # note that since the robot is in a 2D space, only the z-component of the velocity vector
        # matters, and this is angular velocity around the axis perpendicular to the computer screen
        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = 0.0

        # control publisher, used to send the control signals
        self.ControlPublisher = self.create_publisher(
            Twist,
            topic1,
            10
        )

        # subscriber
        # SensorCallbackPose is a function used to receive the Odometry sensor messages
        self.PoseSubscriber = self.create_subscription(
            Odometry,
            topic2,
            self.SensorCallbackPose,
            10
        )

        # SensorCallbackLidar is a function used to receive the Odometry sensor messages
        self.LidarSubscriber = self.create_subscription(
            LaserScan,
            topic3,
            self.SensorCallbackLidar,
            10
        )

        # how often the messages are being sent
        # this is the control frequency
        self.period = 0.05

        # timer object

        # ControlFunction is the function for calculating and sending control actions
        self.timer = self.create_timer(
            self.period,
            self.ControlFunction
        )

    # function used to compute the orientation angle error
    # for computing the angular velocity set point
    def orientationError(self, theta_, thetad_):

        # thetad in 2nd quadrant, theta in 3rd quadrant
        if (thetad_ > np.pi / 2) and (thetad_ <= np.pi):
            if (theta_ > -np.pi) and (theta_ <= -np.pi / 2):
                theta_ = theta_ + 2 * np.pi

        # theta in 2nd quadrant, thetad in 3rd quadrant
        if (theta_ > np.pi / 2) and (theta_ <= np.pi):
            if (thetad_ > -np.pi) and (thetad_ <= -np.pi / 2):
                thetad_ = thetad_ + 2 * np.pi

        errorOrientation = thetad_ - theta_
        return errorOrientation

    def SensorCallbackPose(self, receivedMsg):

        # store the received odometry message
        self.OdometryMsg = receivedMsg

        # record time of the received message
        self.msgOdometryTime = time.time()

    # function for receiving lidar sensor information
    def SensorCallbackLidar(self, receivedMsg):
        self.LidarMsg = receivedMsg

        # record time of the received message
        self.msgLidarTime = time.time()

    # function for calculating and sending control actions
    def ControlFunction(self):

        # to simplify the notation, extract the stored parameters
        # ka - control parameter for the attractive force
        ka = self.kap

        # kr - control parameter for the repulsive force
        kr = self.krp

        # ktheta - control parameter for the orientation controller
        ktheta = self.kthetap

        # gstar - parameter used to limit the influence of repulsive force
        gstar = self.gstarp

        # extract the desired position coordinates - to simplify the notation
        xd = self.xdp
        yd = self.ydp

        # extract the current position and orientation
        # these are xB and yB in the report
        x = self.OdometryMsg.pose.pose.position.x
        y = self.OdometryMsg.pose.pose.position.y

        # extract the quaternion from the message
        quat = self.OdometryMsg.pose.pose.orientation

        # list with quaternion components
        quatl = [quat.x, quat.y, quat.z, quat.w]

        # extract the angle, only the yaw angle is changing
        roll, pitch, yaw = euler_from_quaternion(quatl)

        # this is the orientation angle
        # orientation angle is expressed
        # for angles 0 to 180 degrees in [0, pi] radians
        # for angles 180 to 360 degrees in the interval (-pi, 0)
        # the same applies for atan2 function
        theta = yaw

        # extract the lidar measurements and parameters
        LidarRanges = np.array(self.LidarMsg.ranges)
        # print(LidarRanges)

        # Explanation of LidarRanges:
        # LidarRanges is an array containing distance measurements to obstacle points
        # if the ray is reflected from an obstacle point, the range is finite
        # otherwise the value is infinite
        # typically, lidar ranges look like this
        # LidarRanges=[inf, inf, inf, ... inf, 3.2, 3.21, 3.3, 3.4, inf, inf, inf, ..., inf, 4.2, 4.23]
        # the non-zero entries correspond to the distance to obstacle points, and we need to analyze

        # this is the angle from which the measurements are started
        angle_min = self.LidarMsg.angle_min

        # this is the angle increment of rays - rays are separated by equidistant angles
        angle_increment = self.LidarMsg.angle_increment

        # attractive force
        vectorD = np.array([[x - xd], [y - yd]])
        gradUa = ka * vectorD
        AF = -gradUa

        # extract the indices of measurements that are not infinite
        # infinite values correspond to the measurements that are not reflected
        # and we neglect them
        indices_not_inf = np.where(~np.isinf(LidarRanges))[0]
        # print(indices_not_inf)

        # if there is an obstacle in the lidar range, set this to true
        obstacleYES = ~np.all(np.isinf(indices_not_inf))

        if obstacleYES:

            # here, we need to identify the obstacles from the range measurements
            # we need to segment the indices according to the obstacle they belong to
            # The idea is that obstacles can be identified by identifying inf (non-reflected)
            # rays between them. Consequently, if
            # indices_not_inf=[1 2 3 5 6 7 10 11 12]
            # then, we have three obstacles, corresponding to indices
            # [1,2,3], [5,6,7], [10,11,12]
            # That is, we are searching for a gap in index number greater than 1 to
            # identify an obstacle

            # Calculate the differences between consecutive elements
            diff_array = np.diff(indices_not_inf)

            # Here, we find the indices where the difference is greater than 1
            # Add 1 to get the split points in the original array
            split_indices = np.where(np.abs(diff_array) > 1)[0] + 1

            # split according to the split indices
            # this array will contain subarrays which contain indices of obstacles
            # every subarray correspond to one obstacle and its points
            partitioned_arrays = np.split(indices_not_inf, split_indices)

            # calculate the angles of all rays
            # we need this for verification and debugging
            angles = angle_min + indices_not_inf * angle_increment + theta

            # calculate the position of all points of all obstacles
            # this is done for debugging
            distances = LidarRanges[indices_not_inf]

            # compute obstacle point positions - all positions -
            # this is for tracking the results and debugging
            xo = x * np.ones(distances.shape) + distances * np.cos(angles)
            yo = y * np.ones(distances.shape) + distances * np.sin(angles)

            # compute minimal distances to obstacles and corresponding angles
            # this list contains the minimal distances to obstacles
            # that is, it contains g values
            min_distances = []

            # this list contains the corresponding angles
            min_distances_angles = []

            for i in range(len(partitioned_arrays)):
                tmpArray = LidarRanges[partitioned_arrays[i]]
                min_index = np.argmin(tmpArray)
                min_distances.append(min(tmpArray))
                min_distances_angles.append(
                    angle_min + angle_increment * partitioned_arrays[i][min_index]
                )

            # compute the coordinates of the obstacle point O in the fixed frame
            xo_min = []
            yo_min = []

            for i in range(len(min_distances)):
                xo_min.append(x + min_distances[i] * np.cos(min_distances_angles[i] + theta))
                yo_min.append(y + min_distances[i] * np.sin(min_distances_angles[i] + theta))

            # compute the gradient value for every obstacle
            g_values = []
            gradUr = []

            for i in range(len(min_distances)):
                gradUr_i = np.array([[0], [0]])
                g_val = np.sqrt((x - xo_min[i])**2 + (y - yo_min[i])**2)
                g_values.append(g_val)

                if (g_val <= gstar):
                    # scalar in front of expression
                    pr = kr * (1 / gstar - (1 / (g_values[i]))) * (1 / ((g_values[i])**3))
                    gradUr_i = pr * np.array([[(x - xo_min[i])], [y - yo_min[i]]])

                gradUr.append(gradUr_i)

            # compute the repulsive force
            RF = np.array([[0], [0]])

            for i in range(len(gradUr)):
                RF = RF + gradUr[i]

            RF = -RF

        if (obstacleYES):
            # complete force
            F = AF + RF
        else:
            F = AF

        # calculate the desired orientation
        thetaD = math.atan2(F[1, 0], F[0, 0])
        # atan2 for angles between 0 and 180, returns 0 to pi
        # for angles between 180 and 360 returns -pi to 0

        # calculate the orientation angle error
        # solve the issue with angles in the second and third quadrants
        eorient = self.orientationError(theta, thetaD)

        # if the distance is smaller than a tolerance
        # turn off the controller
        if (np.linalg.norm(vectorD, 2) < self.eps_control):
            thetavel = 0.0
            xvel = 0.0
        else:
            # control

            # if the orientation error is too large, first adjust the angle
            if (np.abs(eorient) > self.eps_orient):
                # adjust the orientation
                thetavel = ktheta * eorient
                # no linear velocity, just pure rotation
                xvel = 0.0
            else:
                # if the orientation error is acceptable
                # then control both orientation and velocity
                thetavel = ktheta * eorient
                xvel = np.linalg.norm(F, 2)

        # set minimal and maximal values for velocity
        # if (np.abs(xvel) < 0.01):
        #     xvel = 0.0099

        if (np.abs(xvel) > 2.6):
            xvel = 1.0

        # print(thetaD)
        # print(eorient)
        # print(F)
        # print(RF)
        # print(thetavel)
        # print(xvel)
        # print(angles)

        # This is the Twist message that we are sending
        # here, we are creating a control velocity actions/command we want to send

        self.controlVel.linear.x = xvel
        # self.controlVel.linear.x = 0.0
        self.controlVel.linear.y = 0.0
        self.controlVel.linear.z = 0.0
        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = thetavel

        print("Sending the control command")

        # publish the control message
        self.ControlPublisher.publish(self.controlVel)

        # print the sensor information on the screen for logging
        print("Received pose:")
        timeDiff = self.msgOdometryTime - self.initialTime

        print(f"Time,x,y,theta:({timeDiff:.3f},{x:.3f},{y:.3f},{theta:.3f})")


def main(args=None):
    # before using rclpy, we need to call the init function
    rclpy.init(args=args)

    # set the coordinates of the desired point
    xd_u = 10.0
    yd_u = -10.0

    # select the control parameters
    ka_u = 2.0
    kr_u = 6.0
    ktheta_u = 1.5
    gstar_u = 0.5

    # in radians
    eps_orient_u = np.pi / 12

    # in meters
    eps_control_u = 0.10

    # create the node
    TestNode = ControllerNode(
        xd_u,
        yd_u,
        ka_u,
        kr_u,
        ktheta_u,
        gstar_u,
        eps_orient_u,
        eps_control_u
    )

    # start the node
    rclpy.spin(TestNode)

    # destroy the node
    TestNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()