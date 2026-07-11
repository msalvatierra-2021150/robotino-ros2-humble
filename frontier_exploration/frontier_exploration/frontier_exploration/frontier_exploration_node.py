#! /usr/bin/env python3

'''
This node is for autonomous exploration. It requests frontier regions from a service and
sends the goal points to the nav2 stack until the the entire environment has been explored.

Reference code: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/
'''
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frontier_interfaces.srv import FrontierGoal
from frontier_exploration.robot_navigator import BasicNavigator, NavigationResult
 
import time
import math

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.goal_pose = PoseStamped()
        self.next_frontier_rank = 0

        self.next_frontier_rank = 0
        self.frontier_retry_count = 0
        self.MAX_RETRIES_PER_FRONTIER = 5
        
        self.cli = self.create_client(FrontierGoal, 'frontier_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = FrontierGoal.Request()
        self.navigator = BasicNavigator()

        self.EXPLORATION_TIME_OUT_SEC = Duration(seconds=1200)
        self.NAV_TO_GOAL_TIMEOUT_SEC = 75
        self.DIST_THRESH_FOR_HEADING_CALC = 0.25

        self.goal_pose = PoseStamped()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()
        self.get_logger().info('Starting frontier exploration...')
        self.explore()

    def explore(self):
        while rclpy.ok():
            # Allow SLAM and Nav2 costmaps to incorporate
            # the scans collected at the previous frontier.
            time.sleep(1.5)

            self.goal_pose = self.get_reachable_goal()

            if self.goal_pose is None:
                self.get_logger().warn(
                    "No reachable frontier currently; retrying."
                )
                time.sleep(2.0)
                continue

            if self.goal_pose == "Done":
                self.get_logger().info(
                    "No frontier currently detected; retrying."
                )
                time.sleep(2.0)
                continue

            self.navigator.goToPose(self.goal_pose)

            canceled_by_explorer = False

            while not self.navigator.isNavComplete():
                feedback = self.navigator.getFeedback()

                if (
                    feedback is not None
                    and Duration.from_msg(feedback.navigation_time)
                    > Duration(seconds=self.NAV_TO_GOAL_TIMEOUT_SEC)
                ):
                    canceled_by_explorer = True

                    self.get_logger().warn(
                        "Navigation timeout; canceling frontier goal."
                    )
                    self.navigator.cancelNav()

                    # Cancellation is asynchronous. Wait until Nav2 confirms completion.
                    while not self.navigator.isNavComplete():
                        time.sleep(0.05)

                    break

            if canceled_by_explorer:
                result = NavigationResult.CANCELED
            else:
                result = self.navigator.getResult()

            self.log_nav_status(result)

            if result == NavigationResult.SUCCEEDED:
                self.next_frontier_rank = 0
            else:
                self.next_frontier_rank += 1
                self.get_logger().warn(
                    f"Frontier navigation failed; trying rank "
                    f"{self.next_frontier_rank}."
                )

    def get_reachable_goal(self):
        rank = self.next_frontier_rank
        reachable = False
        while not reachable:
            goal = self.send_request(rank)
            if goal is None:
                self.next_frontier_rank = 0
                return "Done"

            self.goal_pose = goal
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            # sanity check a valid path exists
            initial_pose = self.get_current_pose()
            if initial_pose is None:
                # Return goal is current pose is unavailble
                return goal
            path = self.navigator.getPath(initial_pose, self.goal_pose)

            # If top 4 frontiers are not reachable, abort
            if path is not None:
                self.next_frontier_rank = rank
                return goal
            elif rank > 3:
                return None
            rank += 1

    def send_request(self, rank):
        self.req.goal_rank = rank
        self.future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        response = self.future.result()

        if response is None:
            return None

        goal = response.goal_pose

        if (
            not math.isfinite(goal.pose.position.x)
            or not math.isfinite(goal.pose.position.y)
        ):
            return None

        return goal

    def get_current_pose(self) -> PoseStamped:
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time(), Duration(seconds=0.5))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform odom to base_link: {ex}')
            self.get_logger().warn('Current pose unavailable.')
            return None
            
        p = PoseStamped()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        return p
    
    def set_goal_heading(self):
        curr_pose = self.get_current_pose()

        if curr_pose is None:
            return
        
        # Set goal orientation to current heading
        self.goal_pose.pose.orientation.x = curr_pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = curr_pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = curr_pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = curr_pose.pose.orientation.w
    
    def log_nav_status(self, result):
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().error('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()   
    frontier_explorer.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()