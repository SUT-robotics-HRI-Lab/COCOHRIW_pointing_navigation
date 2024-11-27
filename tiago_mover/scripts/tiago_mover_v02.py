#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mozek_decider.msg import AngleDistance  # Import the custom message type
from tf.transformations import quaternion_from_euler  # Import quaternion conversion function


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
        angle2 = msg.angle2  # Angle in degrees to face the operator

        # Log the received values
        rospy.loginfo(f"Received destination: x={dest_x}, y={dest_y}, angle2={angle2}")

        # Send the goal with orientation based on angle2
        self.send_goal(dest_x, dest_y, angle2)

    def send_goal(self, x, y, angle_degrees, frame_id="map"):
        """
        Sends a goal to the /move_base_simple/goal topic.

        :param x: X coordinate of the goal
        :param y: Y coordinate of the goal
        :param angle_degrees: Rotation in degrees around the z-axis to face the operator
        :param frame_id: Reference frame for the coordinates (default: "map")
        """
        # Convert angle from degrees to radians
        angle_radians = angle_degrees * (3.14159265359 / 180.0)

        # Create a quaternion from the yaw angle (rotation around z-axis)
        quaternion = quaternion_from_euler(0, 0, angle_radians)

        # Create a PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = frame_id
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]

        # Publish the goal
        self.goal_publisher.publish(goal_msg)
        rospy.loginfo(f"Goal sent to x={x}, y={y}, angle={angle_degrees} degrees in frame: {frame_id}")

if __name__ == "__main__":
    try:
        # Initialize the node
        node = MoveBaseNode()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveBaseNode terminated.")

