#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mozek_decider.msg import AngleDistance  # Import the custom message type

class MoveBaseNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_base_node', anonymous=True)

        # Publisher to send goals to the /move_base_simple/goal topic
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscriber to the /angle_distance_topic
        rospy.Subscriber('/angle_distance_topic', AngleDistance, self.angle_distance_callback)

        rospy.loginfo("MoveBaseNode initialized. Waiting for messages on /angle_distance_topic...")

    def angle_distance_callback(self, msg):
        """
        Callback function for the /angle_distance_topic subscriber.

        :param msg: The message of type mozek_decider/AngleDistance
        """
        dest_x = msg.destX
        dest_y = msg.destY

        # Log the received coordinates
        rospy.loginfo(f"Received destination: x={dest_x}, y={dest_y}")

        # Send the goal
        self.send_goal(dest_x, dest_y)

    def send_goal(self, x, y, frame_id="base_link"):
        """
        Sends a goal to the /move_base_simple/goal topic.

        :param x: X coordinate of the goal
        :param y: Y coordinate of the goal
        :param frame_id: Reference frame for the coordinates (default: "map")
        """
        # Create a PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = frame_id
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # Default orientation (facing forward)

        # Publish the goal
        self.goal_publisher.publish(goal_msg)
        rospy.loginfo(f"Goal sent to x={x}, y={y} in frame: {frame_id}")

if __name__ == "__main__":
    try:
        # Initialize the node
        node = MoveBaseNode()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveBaseNode terminated.")
