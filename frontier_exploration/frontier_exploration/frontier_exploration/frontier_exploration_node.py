#!/usr/bin/env python3

"""
Frontier exploration controlled by the eMDB selected-policy topic.

Exploration runs only while /robotino/emdb/selected_policy contains:
    policy_id: 0
    policy_name: continuing_exploration

When another policy is selected, the current Nav2 goal is canceled and
frontier exploration pauses until continuing_exploration is selected again.
"""

import math
import threading
import time
from typing import Optional, Tuple, Union

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_interfaces.srv import FrontierGoal
from frontier_exploration.robot_navigator import (
    BasicNavigator,
    NavigationResult,
)
from robotino_emdb_interfaces.msg import RobotinoSelectedPolicy


class FrontierExplorer(Node):
    """Request reachable frontier goals and send them to Nav2."""

    POLICY_TOPIC = "/robotino/emdb/selected_policy"
    EXPLORATION_POLICY_ID = 0
    EXPLORATION_POLICY_NAME = "continue_exploring"

    FRONTIER_SERVICE = "frontier_pose"
    MAP_FRAME = "map"
    ROBOT_FRAME = "base_link"

    MAX_FRONTIER_RANK = 3
    NAVIGATION_TIMEOUT_SEC = 75.0
    FRONTIER_REQUEST_TIMEOUT_SEC = 5.0
    RETRY_DELAY_SEC = 2.0
    SCAN_UPDATE_DELAY_SEC = 1.5
    CANCEL_WAIT_SEC = 5.0

    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        # Worker-state flags.
        self.exploration_enabled = threading.Event()
        self.shutdown_requested = threading.Event()

        self.next_frontier_rank = 0

        # Frontier service client.
        self.frontier_client = self.create_client(
            FrontierGoal,
            self.FRONTIER_SERVICE,
        )

        # TF listener used to obtain the robot pose in the map frame.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Nav2 helper.
        self.navigator = BasicNavigator()

        # eMDB selected-policy subscriber.
        self.policy_subscriber = self.create_subscription(
            RobotinoSelectedPolicy,
            self.POLICY_TOPIC,
            self.selected_policy_callback,
            10,
        )

        # Run blocking frontier and navigation work outside ROS callbacks.
        self.worker_thread = threading.Thread(
            target=self.exploration_worker,
            name="frontier_exploration_worker",
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            "Frontier explorer initialized. Waiting for "
            "policy_id=0 and policy_name='continue_exploring'."
        )

    # ------------------------------------------------------------------
    # Policy control
    # ------------------------------------------------------------------

    def selected_policy_callback(
        self,
        msg: RobotinoSelectedPolicy,
    ) -> None:
        """Enable or pause exploration according to the selected policy."""

        policy_id = int(msg.policy_id)
        policy_name = str(msg.policy_name).strip().lower()

        exploration_selected = (
            policy_id == self.EXPLORATION_POLICY_ID
            and policy_name == self.EXPLORATION_POLICY_NAME
        )

        if exploration_selected:
            if not self.exploration_enabled.is_set():
                self.next_frontier_rank = 0
                self.exploration_enabled.set()

                self.get_logger().info(
                    "continuing_exploration selected. "
                    "Frontier exploration enabled."
                )
            return

        if self.exploration_enabled.is_set():
            self.exploration_enabled.clear()

            self.get_logger().info(
                f"Policy changed to id={policy_id}, "
                f"name='{msg.policy_name}'. "
                "Frontier exploration paused."
            )

    def should_explore(self) -> bool:
        """Return True while exploration is enabled and ROS is running."""

        return (
            rclpy.ok()
            and not self.shutdown_requested.is_set()
            and self.exploration_enabled.is_set()
        )

    # ------------------------------------------------------------------
    # Exploration worker
    # ------------------------------------------------------------------

    def exploration_worker(self) -> None:
        """Continuously request and navigate to frontiers while enabled."""

        while rclpy.ok() and not self.shutdown_requested.is_set():
            # Wait until eMDB selects continuing_exploration.
            if not self.exploration_enabled.wait(timeout=0.25):
                continue

            if not self.should_explore():
                continue

            if not self.wait_for_frontier_service():
                continue

            # Give SLAM and costmaps time to process recent scans.
            if not self.interruptible_sleep(self.SCAN_UPDATE_DELAY_SEC):
                continue

            goal_pose = self.get_reachable_frontier()

            if not self.should_explore():
                continue

            if goal_pose is None:
                self.get_logger().info(
                    "No reachable frontier found. Retrying."
                )
                self.interruptible_sleep(self.RETRY_DELAY_SEC)
                continue

            result, cancellation_reason = self.navigate_to_goal(goal_pose)

            if cancellation_reason == "policy":
                self.get_logger().info(
                    "Navigation canceled because eMDB changed policy."
                )
                continue

            if cancellation_reason == "shutdown":
                break

            self.log_navigation_result(result)

            if result == NavigationResult.SUCCEEDED:
                self.next_frontier_rank = 0
            else:
                self.next_frontier_rank += 1

                if self.next_frontier_rank > self.MAX_FRONTIER_RANK:
                    self.next_frontier_rank = 0

                self.get_logger().warn(
                    "Frontier navigation did not succeed. "
                    f"Next requested rank: {self.next_frontier_rank}."
                )

    def wait_for_frontier_service(self) -> bool:
        """Wait for the frontier service without blocking policy changes."""

        while self.should_explore():
            if self.frontier_client.wait_for_service(timeout_sec=1.0):
                return True

            self.get_logger().info(
                f"Waiting for frontier service "
                f"'{self.FRONTIER_SERVICE}'..."
            )

        return False

    # ------------------------------------------------------------------
    # Frontier selection
    # ------------------------------------------------------------------

    def get_reachable_frontier(self) -> Optional[PoseStamped]:
        """Return the first reachable frontier from the configured ranks."""

        rank = self.next_frontier_rank

        while self.should_explore() and rank <= self.MAX_FRONTIER_RANK:
            goal = self.request_frontier(rank)

            if not self.should_explore():
                return None

            if goal is None:
                self.next_frontier_rank = 0
                return None

            goal.header.frame_id = self.MAP_FRAME
            goal.header.stamp = self.get_clock().now().to_msg()

            current_pose = self.get_current_pose()

            if current_pose is None:
                self.get_logger().warn(
                    "Robot pose is unavailable. Using the frontier "
                    "without checking its path first."
                )
                self.next_frontier_rank = rank
                return goal

            path = self.navigator.getPath(current_pose, goal)

            if path is not None and len(path.poses) > 0:
                self.next_frontier_rank = rank
                return goal

            self.get_logger().warn(
                f"Frontier rank {rank} is unreachable."
            )
            rank += 1

        self.next_frontier_rank = 0
        return None

    def request_frontier(self, rank: int) -> Optional[PoseStamped]:
        """Request one ranked frontier pose from the frontier service."""

        request = FrontierGoal.Request()
        request.goal_rank = rank

        future = self.frontier_client.call_async(request)
        deadline = (
            time.monotonic() + self.FRONTIER_REQUEST_TIMEOUT_SEC
        )

        # The MultiThreadedExecutor completes the service future while
        # this worker thread waits.
        while rclpy.ok() and not future.done():
            if not self.should_explore():
                return None

            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"Frontier request for rank {rank} timed out."
                )
                return None

            time.sleep(0.05)

        if not future.done():
            return None

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"Frontier service request failed: {error}"
            )
            return None

        if response is None:
            return None

        goal = response.goal_pose

        if (
            not math.isfinite(goal.pose.position.x)
            or not math.isfinite(goal.pose.position.y)
        ):
            self.get_logger().warn(
                f"Frontier rank {rank} returned invalid coordinates."
            )
            return None

        return goal

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def navigate_to_goal(
        self,
        goal_pose: PoseStamped,
    ) -> Tuple[NavigationResult, Optional[str]]:
        """Navigate to a frontier and stop if the policy changes."""

        self.navigator.goToPose(goal_pose)

        while rclpy.ok() and not self.navigator.isNavComplete():
            if not self.should_explore():
                self.cancel_navigation_and_wait()
                return NavigationResult.CANCELED, "policy"

            feedback = self.navigator.getFeedback()

            if feedback is not None:
                navigation_time = Duration.from_msg(
                    feedback.navigation_time
                )

                if navigation_time > Duration(
                    seconds=self.NAVIGATION_TIMEOUT_SEC
                ):
                    self.get_logger().warn(
                        "Navigation timeout. Canceling frontier goal."
                    )
                    self.cancel_navigation_and_wait()
                    return NavigationResult.CANCELED, "timeout"

            time.sleep(0.05)

        if not rclpy.ok() or self.shutdown_requested.is_set():
            return NavigationResult.CANCELED, "shutdown"

        return self.navigator.getResult(), None

    def cancel_navigation_and_wait(self) -> None:
        """Cancel the current Nav2 goal and briefly wait for completion."""

        self.navigator.cancelNav()
        deadline = time.monotonic() + self.CANCEL_WAIT_SEC

        while (
            rclpy.ok()
            and not self.navigator.isNavComplete()
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # Robot pose
    # ------------------------------------------------------------------

    def get_current_pose(self) -> Optional[PoseStamped]:
        """Return the robot pose in the map frame."""

        try:
            transform = self.tf_buffer.lookup_transform(
                self.MAP_FRAME,
                self.ROBOT_FRAME,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Could not transform {self.MAP_FRAME} to "
                f"{self.ROBOT_FRAME}: {error}"
            )
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation

        return pose

    # ------------------------------------------------------------------
    # Utilities and shutdown
    # ------------------------------------------------------------------

    def interruptible_sleep(self, duration_sec: float) -> bool:
        """Sleep while still allowing a policy change to interrupt."""

        deadline = time.monotonic() + duration_sec

        while time.monotonic() < deadline:
            if not self.should_explore():
                return False

            remaining = deadline - time.monotonic()
            time.sleep(min(0.1, remaining))

        return True

    def log_navigation_result(self, result: NavigationResult) -> None:
        """Log the result returned by the Nav2 helper."""

        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info("Frontier goal succeeded.")
        elif result == NavigationResult.CANCELED:
            self.get_logger().info("Frontier goal was canceled.")
        elif result == NavigationResult.FAILED:
            self.get_logger().warn("Frontier goal failed.")
        else:
            self.get_logger().error(
                f"Unknown navigation result: {result}"
            )

    def stop_worker(self) -> None:
        """Stop and join the exploration worker thread."""

        self.shutdown_requested.set()

        # Wake the worker if it is waiting for exploration to be enabled.
        self.exploration_enabled.set()

        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=3.0)


def main(args=None) -> None:
    rclpy.init(args=args)

    frontier_explorer = FrontierExplorer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(frontier_explorer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        frontier_explorer.stop_worker()
        executor.remove_node(frontier_explorer)
        executor.shutdown()

        frontier_explorer.navigator.destroy_node()
        frontier_explorer.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
