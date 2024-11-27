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
        self.move_base_client = actionlib.SimpleActionClient('/goto/reach', MoveBaseAction)

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
        dest_x = msg.distance/1000.0
        dest_y = msg.destY
        angle1 = msg.angle1
        angle2 = msg.angle2  # Angle in degrees to face the operator

        # Log the received values
        rospy.loginfo(f"Received destination: x={dest_x}, y={dest_y}, angle2={angle2}")

        # Send the goal with orientation based on angle2
        self.send_goal(dest_x, dest_y, angle1, angle2)

    def send_goal(self, x, y, angle1_degrees, angle2_degrees, frame_id="base_link"):
        """
        Sends a goal to the /move_base action server.

        :param x: X coordinate of the goal
        :param y: Y coordinate of the goal
        :param angle_degrees: Rotation in degrees around the z-axis to face the operator
        :param frame_id: Reference frame for the coordinates (default: "map")
        """
        # Convert angle from degrees to radians
        angle1_radians = angle1_degrees * (3.14159265359 / 180.0)
        angle2_radians = angle2_degrees * (3.14159265359 / 180.0)

        # Create a quaternion from the yaw angle (rotation around z-axis)
        quaternion1 = quaternion_from_euler(0, 0, -angle1_radians)
        quaternion2 = quaternion_from_euler(0, 0, -angle2_radians)

        # Create a MoveBaseGoal message
        goal1 = MoveBaseGoal()
        goal1.target_pose.header.stamp = rospy.Time.now()
        goal1.target_pose.header.frame_id = frame_id
        goal1.target_pose.pose.position = Point(x=0, y=0.0, z=0.0)
        goal1.target_pose.pose.orientation = Quaternion(
            x=quaternion1[0], y=quaternion1[1], z=quaternion1[2], w=quaternion1[3]
        )

        # Send the goal to the action server
        rospy.loginfo(f"Sending goal 1 to /goto/reach/goal: x=0, y=0, angle={angle1_degrees} degrees")
        
        rospy.set_param("/robot_drive_lock", True)
        rospy.loginfo("ROBOT DRIVE LOCK: LOCKED")
        
        self.move_base_client.send_goal(goal1)

        # Wait for the result
        rospy.loginfo("Waiting for result from /move_base action server...")
        self.move_base_client.wait_for_result()

        # Check the result
        result = self.move_base_client.get_result()
        if result:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach the goal.")
            
            
        # Create a MoveBaseGoal message
        goal2 = MoveBaseGoal()
        goal2.target_pose.header.stamp = rospy.Time.now()
        goal2.target_pose.header.frame_id = frame_id
        goal2.target_pose.pose.position = Point(x=x, y=0.0, z=0.0)
        goal2.target_pose.pose.orientation = Quaternion(
            x=0, y=0, z=0, w=1.0
        )

        # Send the goal to the action server
        rospy.loginfo(f"Sending goal 2 to /goto/reach/goal: x={x}, y={y}, angle=0 degrees")
        
        
        self.move_base_client.send_goal(goal2)

        # Wait for the result
        rospy.loginfo("Waiting for result from /move_base action server...")
        self.move_base_client.wait_for_result()

        # Check the result
        result = self.move_base_client.get_result()
        if result:
            rospy.loginfo("Goal reached successfully!")

        else:
            rospy.logwarn("Failed to reach the goal.")

            
        # Create a MoveBaseGoal message
        goal3 = MoveBaseGoal()
        goal3.target_pose.header.stamp = rospy.Time.now()
        goal3.target_pose.header.frame_id = frame_id
        goal3.target_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        goal3.target_pose.pose.orientation = Quaternion(
            x=quaternion2[0], y=quaternion2[1], z=quaternion2[2], w=quaternion2[3]
        )

        # Send the goal to the action server
        rospy.loginfo(f"Sending goal3 to /goto/reach/goal: x=0, y=0, angle={angle2_degrees} degrees")
        
        
        self.move_base_client.send_goal(goal3)

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

