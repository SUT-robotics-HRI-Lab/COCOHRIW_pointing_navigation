#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mozek_decider.msg import AngleDistance  # Import the custom message type
from tf.transformations import quaternion_from_euler  # Import quaternion conversion function


class MoveBaseNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_base_node', anonymous=True)

        # Action client to send goals to the /move_base action server
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Wait for the action server to start
        rospy.loginfo("Waiting for /move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to /move_base action server.")

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

    def send_goal(self, x, y, angle_degrees, frame_id="base_link"):
        """
        Sends a goal to the /move_base action server.

        :param x: X coordinate of the goal
        :param y: Y coordinate of the goal
        :param angle_degrees: Rotation in degrees around the z-axis to face the operator
        :param frame_id: Reference frame for the coordinates (default: "map")
        """
        # Convert angle from degrees to radians
        angle_radians = angle_degrees * (3.14159265359 / 180.0)

        # Create a quaternion from the yaw angle (rotation around z-axis)
        quaternion = quaternion_from_euler(0, 0, angle_radians)

        # Create a MoveBaseGoal message
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose.position = Point(x=x, y=y, z=0.0)
        goal.target_pose.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
        )

        # Send the goal to the action server
        rospy.loginfo(f"Sending goal to /move_base: x={x}, y={y}, angle={angle_degrees} degrees")
        
        rospy.set_param("/robot_drive_lock", True)
        rospy.loginfo("ROBOT DRIVE LOCK: LOCKED")
        
        self.move_base_client.send_goal(goal)

        # Wait for the result
        rospy.loginfo("Waiting for result from /move_base action server...")
        self.move_base_client.wait_for_result()

        # Check the result
        result = self.move_base_client.get_result()
        if result:
            rospy.loginfo("Goal reached successfully!")
            rospy.set_param("/robot_drive_lock", False)
            rospy.loginfo("ROBOT DRIVE LOCK: OPENED")
        else:
            rospy.logwarn("Failed to reach the goal.")
            rospy.set_param("/robot_drive_lock", False)
            rospy.loginfo("ROBOT DRIVE LOCK: OPENED")

if __name__ == "__main__":
    try:
        # Initialize the node
        node = MoveBaseNode()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveBaseNode terminated.")

