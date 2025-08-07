import argparse
import math
import numpy as np
import sys

import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def get_euclidean_distance(p1, p2):
    if len(p1) == 2:
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    else:
        return math.sqrt(
            (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2
        )


class GoalPublisher:
    def __init__(self, d_min, d_dim, goals):
        self.d_min = d_min
        self.d_dim = d_dim
        if self.d_dim != 2 and self.d_dim != 3:
            print("Invalid dimension for distance metric. Must be 2 or 3")
            sys.exit(1)
        self.goals = goals
        self.n_goals = np.shape(goals)[1]
        self.goal_idx = -1
        print("Goals: {}".format(self.goals))

        # Initialize ROS publishers and subscribers
        self.publisher = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/falcon/ground_truth/odometry", Odometry, self.state_callback)

        # Initialize goal message
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = "map"
        self.goal_msg.pose.position.x = 0
        self.goal_msg.pose.position.y = 0
        self.goal_msg.pose.position.z = 1.6
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = 0
        self.goal_msg.pose.orientation.w = 1

        # Sleep some time before publishing first goal
        rospy.sleep(1)

        # Publish first goal
        self.publish_goal()

    def publish_goal(self):
        # # Increase goal index and exit if we run out of goals
        if self.goal_idx + 1 >= self.n_goals:
            print("All goals reached! Exiting")
            rospy.signal_shutdown("All goals reached! Exiting")
        else:
            self.goal_idx += 1

        # Select current goal
        goal = self.goals[:, self.goal_idx]

        # Convert ZYX Euler angles to quaternion for ROS
        q = tft.quaternion_from_euler(0, 0, goal[3], axes="rzyx")

        # Create goal message
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.pose.position.x = goal[0]
        self.goal_msg.pose.position.y = goal[1]
        self.goal_msg.pose.position.z = goal[2]
        self.goal_msg.pose.orientation.x = q[0]
        self.goal_msg.pose.orientation.y = q[1]
        self.goal_msg.pose.orientation.z = q[2]
        self.goal_msg.pose.orientation.w = q[3]

        # Publish goal
        self.publisher.publish(self.goal_msg)

        # Print goal confirmation
        print(
            "Goal {} published: {{x: {}, y: {}, z: {}, psi: {}}}".format(
                self.goal_idx, goal[0], goal[1], goal[2], goal[3]
            )
        )

    def state_callback(self, msg):
        if (
            self.d_dim == 2
            and get_euclidean_distance(
                [msg.pose.pose.position.x, msg.pose.pose.position.y],
                self.goals[:2, self.goal_idx],
            )
            < self.d_min
        ):
            print(
                "Reached goal {} within distance {} m".format(self.goal_idx, self.d_min)
            )
            self.publish_goal()
        elif (
            self.d_dim == 3
            and get_euclidean_distance(
                [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ],
                self.goals[:3, self.goal_idx],
            )
            < self.d_min
        ):
            print(
                "Reached goal {} within distance {} m".format(self.goal_idx, self.d_min)
            )
            self.publish_goal()


if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser(description="Publish a list of goals.")
    parser.add_argument(
        "--d_min", type=float, required=True, help="the minimum distance to goal"
    )
    parser.add_argument(
        "--d_dim",
        default=3,
        type=int,
        help="the number of dimensions in which to evaluate the distance metric",
    )
    parser.add_argument(
        "--goals",
        metavar="G",
        type=float,
        nargs="+",
        help="a list of goals (x, y, z, yaw)",
    )
    args = parser.parse_args()

    # Pass arguments to goal publisher
    d_min = args.d_min
    d_dim = args.d_dim
    goals_list = list(zip(*[iter(args.goals)] * 4))
    goals_list = np.array(goals_list)
    goals_list = goals_list.T

    # Initialize ROS node
    rospy.init_node("goal_publisher")

    # Initialize goal publisher
    GoalPublisher(d_min, d_dim, goals_list)

    # Spin ROS
    rospy.spin()
