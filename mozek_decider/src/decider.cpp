#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense> // For matrix operations

#include "intersection_library.hpp"

#include <mozek_decider/AngleDistance.h>

// Function to convert a 4x4 transformation matrix to a tf::Transform
tf::Transform matrixToTf(const Eigen::Matrix4d& matrix) {
    tf::Vector3 translation(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    tf::Matrix3x3 rotation(
        matrix(0, 0), matrix(0, 1), matrix(0, 2),
        matrix(1, 0), matrix(1, 1), matrix(1, 2),
        matrix(2, 0), matrix(2, 1), matrix(2, 2)
    );
    tf::Quaternion quaternion;
    rotation.getRotation(quaternion);

    tf::Transform transform;
    transform.setOrigin(translation);
    transform.setRotation(quaternion);
    return transform;
}

// Function to transform a vector using a tf::Transform
tf::Vector3 transformVector(const tf::Transform& transform, const tf::Vector3& source_vector) {
    return transform * source_vector;
}

double sind(double degrees) {
    return std::sin(degrees * M_PI / 180.0);
}

double cosd(double degrees) {
    return std::cos(degrees * M_PI / 180.0); // Convert degrees to radians
}

double computeAngleOfVectorsInDegrees (Eigen::Vector3d  vec1,Eigen::Vector3d  vec2)
{
    return acos ( vec1.dot ( vec2 ) / ( vec1.norm() * vec2.norm() ) ) * 180.0 / M_PI;
}

void pclPointToTfVector3(const pcl::PointXYZ& pcl_point, tf::Vector3& tf_vector) {
    tf_vector.setX(pcl_point.x);
    tf_vector.setY(pcl_point.y);
    tf_vector.setZ(pcl_point.z);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const tf::Transform& transform,ros::Publisher pub) {
    // Convert PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Print information about the received PointCloud
    //ROS_INFO("Received PointCloud with %lu points", cloud.points.size());

    // Iterate through the points and print their coordinates
 /*   for (const auto& point : cloud.points) {
        ROS_INFO("Point: [x: %f, y: %f, z: %f]", point.x, point.y, point.z);
    }*/

    tf::Vector3 tempTfVec;
    tf::Vector3 robFrameVec;


    //elbow
    pclPointToTfVector3(cloud.points[1], tempTfVec);
    robFrameVec = transformVector(transform, tempTfVec);
    IntersectionLibrary::Vector3 linePoint1(robFrameVec.getX(), robFrameVec.getY(), robFrameVec.getZ());

    //wrist
    //ROS_INFO("Wrist right: [x: %f, y: %f, z: %f]", cloud.points[2].x, cloud.points[2].y, cloud.points[2].z);
    pclPointToTfVector3(cloud.points[2], tempTfVec);
    robFrameVec = transformVector(transform, tempTfVec);
    //ROS_INFO("Wrist right transformed: [x: %f, y: %f, z: %f]", robFrameVec.getX(), robFrameVec.getY(), robFrameVec.getZ());
    IntersectionLibrary::Vector3 linePoint2(robFrameVec.getX(), robFrameVec.getY(), robFrameVec.getZ());

    IntersectionLibrary::IntersectionResult destPoint = IntersectionLibrary::intersectLinePlane(
    linePoint1, linePoint2, IntersectionLibrary::Vector3(0.0,0.0,0.0), IntersectionLibrary::Vector3(0.0,0.0,5));

    bool lock_param_value;
    ros::param::get("/robot_drive_lock", lock_param_value);

    if(!lock_param_value)
      {
    ROS_INFO("DEST POINT: [x: %f, y: %f, z: %f] in meters", std::get<1>(destPoint).x, std::get<1>(destPoint).y, std::get<1>(destPoint).z);
    }



    //lava ruka nad hlavou (v suradniciach kinectu, cize y os, ale opacne, lebo ukazuje dolu)
    // a este nejake bonusove podmienky, aby neboli neplatne info o jointoch
    if(cloud.points[4].y < cloud.points[0].y && cloud.points[4].x != 0.0 && cloud.points[0].x != 0.0 && !lock_param_value)
    {
        ROS_INFO("ROBOT DRIVE LOCK: OPENED");
        ROS_INFO("Driving....");ROS_INFO("Driving....");ROS_INFO("Driving....");
        mozek_decider::AngleDistance angle_distance_msg;

        //head
    	pclPointToTfVector3(cloud.points[0], tempTfVec);
    	robFrameVec = transformVector(transform, tempTfVec);

        Eigen::Vector3d  operatorVec(robFrameVec.getX(), robFrameVec.getY(),0.0);
        Eigen::Vector3d  driveVec(std::get<1>(destPoint).x,std::get<1>(destPoint).y,std::get<1>(destPoint).z);
        Eigen::Vector3d  robotAxisVec(50.0,0.0,0.0);

        // uhly riesim tu tak, ze dolava je zaporny, doprava kladny
		// prvy uhol - robot vs miesto ukazovania
        if(driveVec.y() > 0.0)
        {
          	ROS_INFO("A");
        	angle_distance_msg.angle1 = -computeAngleOfVectorsInDegrees (driveVec, robotAxisVec);  // Placeholder value for angle1 deg
        }
		else
        {
         	ROS_INFO("B");
			angle_distance_msg.angle1 = computeAngleOfVectorsInDegrees (driveVec, robotAxisVec);
		}
		                // druhuy uhol - miesto ukazovania vs operator
        if(driveVec.y() > 0.0)
        {
          ROS_INFO("A");
            angle_distance_msg.angle2 = computeAngleOfVectorsInDegrees ( -driveVec, -operatorVec + driveVec);  // Placeholder value for angle2 deg
        }
        else
        {
          ROS_INFO("B");
          angle_distance_msg.angle2 = -(computeAngleOfVectorsInDegrees ( -driveVec, -operatorVec + driveVec));
		}

        angle_distance_msg.distance = driveVec.norm()*1000.0; // Placeholder value for distance mm
		angle_distance_msg.destX = driveVec.x();
		angle_distance_msg.destY = driveVec.y();

        // Log the message being sent
        ROS_INFO("Publishing: angle1: %f deg, angle2: %f deg , distance: %f mm, destX %f m, destY %f m",
                 angle_distance_msg.angle1,
                 angle_distance_msg.angle2,
                 angle_distance_msg.distance,
                        angle_distance_msg.destX,
                         angle_distance_msg.destY );

        // Publish the AngleDistance message, uhly su v stupnoch imho
        pub.publish(angle_distance_msg);

        ros::param::set("/robot_drive_lock", true);
        //ROS_INFO("ROBOT DRIVE LOCK: LOCKED");


    }
}




int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mozek_decider");
    ros::NodeHandle nh;

    // Input a 4x4 transformation matrix
    Eigen::Matrix4d transformation_matrix;
    double alpha = 3.0; //uhol kinectu do hora natoceny, znamienko zaporne
    double tx = 0.28, tz = 1.26;


    transformation_matrix << 0.0, -sind(alpha),  cosd(alpha),   tx ,   // Row 1
                              -1.0, 0.0, 0.0, 0.0,   // Row 2
                              0.0, -cosd(alpha),  -sind(alpha), tz,   // Row 3
                              0.0, 0.0,    0.0,    1.0;    // Row 4 (Homogeneous row)

    // Convert the matrix to a tf::Transform
    tf::Transform transform = matrixToTf(transformation_matrix);

    ros::Publisher pub = nh.advertise<mozek_decider::AngleDistance>("angle_distance_topic", 10);



    // Subscribe to the PointCloud2 topic
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("azure_points", 10, pointCloudCallback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "azure_points",
        10,
        boost::bind(&pointCloudCallback, _1, boost::cref(transform),pub)
    );

    // Spin to keep the node active and processing callbacks
    ros::spin();

    return 0;
}
