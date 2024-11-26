#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point32.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//kinect azure stuff
#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>



k4a_device_t device = nullptr;
k4abt_tracker_t tracker = nullptr;

struct InputSettings
{
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
    bool Offline = false;
    std::string FileName;
    std::string ModelPath;
};



void SetupDevice(InputSettings inputSettings)
{
    //k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    printf("debug1\n");
    //k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    //myso trackerConfig.model_path = inputSettings.ModelPath.c_str();
    printf("debug1.5\n");
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");
    printf("debug2\n");
}



int main(int argc, char **argv) {
    //toto mi treba zo simple_3d_viewera
    InputSettings inputSettings; //azure kinect
    //toto tu uz len treba dat do tej slucky publishera a vybavene
    SetupDevice(inputSettings); //azure kinect



    ros::init(argc, argv, "kinect_publisher");
    ros::NodeHandle nh;

    // Create a publisher for PointCloud2 messages
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("azure_points", 1);

    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 10; // 4 points
    cloud.height = 1; // 1-D cloud
    cloud.points.resize(cloud.width * cloud.height);
    // Define some fixed points
    for(int i = 0; i <10; i++)
    {
        cloud.points[i].x = 0.0; cloud.points[i].y = 0.0; cloud.points[i].z = 0.0; // Point 1 (Origin)
    }


    ros::Rate loop_rate(5); // 1 Hz
    while (ros::ok()) {



        ////kinect azure stuff
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/
            // Obtain original capture that generates the body tracking result
            k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
            k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);

/*
            // Read body index map and assign colors
            k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
            const uint8_t *bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
            for (int i = 0; i < depthWidth * depthHeight; i++)
            {
                uint8_t bodyIndex = bodyIndexMapBuffer[i];
                if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND) {
                    uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
                    pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
                }
            }
            k4a_image_release(bodyIndexMap);
*/
            uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
            for (uint32_t i = 0; i < numBodies; i++) {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton),
                       "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.4f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0.1f;

                /*
                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++) {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
                        const k4a_float3_t &jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t &jointOrientation = body.skeleton.joints[joint].orientation;

                        if (joint == K4ABT_JOINT_ELBOW_RIGHT )
                            printf("%f %f %f,", body.skeleton.joints[joint].position.xyz.x,
                                   body.skeleton.joints[joint].position.xyz.y,
                                   body.skeleton.joints[joint].position.xyz.z);
                    }
                }
                printf("KEKET\n");*/

                //HEAD
                cloud.points[0].x = body.skeleton.joints[K4ABT_JOINT_HEAD].position.xyz.x/1000.0;
                cloud.points[0].y = body.skeleton.joints[K4ABT_JOINT_HEAD].position.xyz.y/1000.0;
                cloud.points[0].z = body.skeleton.joints[K4ABT_JOINT_HEAD].position.xyz.z/1000.0;
                //RIGHT ELBOW
                cloud.points[1].x = body.skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position.xyz.x/1000.0;
                cloud.points[1].y = body.skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position.xyz.y/1000.0;
                cloud.points[1].z = body.skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position.xyz.z/1000.0;
                //RIGHT HAND WRIST
                cloud.points[2].x = body.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position.xyz.x/1000.0;
                cloud.points[2].y = body.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position.xyz.y/1000.0;
                cloud.points[2].z = body.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position.xyz.z/1000.0;
                //LEFT ELBOW
                cloud.points[3].x = body.skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position.xyz.x/1000.0;
                cloud.points[3].y = body.skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position.xyz.y/1000.0;
                cloud.points[3].z = body.skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position.xyz.z/1000.0;
                //LEFT HAND TIP
                cloud.points[4].x = body.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position.xyz.x/1000.0;
                cloud.points[4].y = body.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position.xyz.y/1000.0;
                cloud.points[4].z = body.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position.xyz.z/1000.0;
                //PELVIS - panva
                cloud.points[5].x = body.skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.x/1000.0;
                cloud.points[5].y = body.skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.y/1000.0;
                cloud.points[5].z = body.skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.z/1000.0;
                //RIGHT KNEE
                cloud.points[6].x = body.skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position.xyz.x/1000.0;
                cloud.points[6].y = body.skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position.xyz.y/1000.0;
                cloud.points[6].z = body.skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position.xyz.z/1000.0;
                //LEFT KNEE
                cloud.points[7].x = body.skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position.xyz.x/1000.0;
                cloud.points[7].y = body.skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position.xyz.y/1000.0;
                cloud.points[7].z = body.skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position.xyz.z/1000.0;
                //RIGHT ANKLE
                cloud.points[8].x = body.skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position.xyz.x/1000.0;
                cloud.points[8].y = body.skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position.xyz.y/1000.0;
                cloud.points[8].z = body.skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position.xyz.z/1000.0;
                //LEFT ANKLE
                cloud.points[9].x = body.skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position.xyz.x/1000.0;
                cloud.points[9].y = body.skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position.xyz.y/1000.0;
                cloud.points[9].z = body.skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position.xyz.z/1000.0;
                /*
                cloud.points[1].x = 1.0; cloud.points[1].y = 0.0; cloud.points[1].z = 0.0; // Point 2
                cloud.points[2].x = 0.0; cloud.points[2].y = 2.0; cloud.points[2].z = 0.0; // Point 3
                cloud.points[3].x = 0.0; cloud.points[3].y = 0.0; cloud.points[3].z = -1.0; // Point 4*/
            }
            k4a_capture_release(originalCapture);
            k4a_image_release(depthImage);
            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }




        //// ROS stuff - Convert to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);

        // Set the header information
        output.header.frame_id = "base_link"; // or whatever frame you want
        output.header.stamp = ros::Time::now(); // Set the timestamp

        // Publish the point cloud
        //len, ked neni nulovy
        if(cloud.points[0].x != 0.0 && cloud.points[1].x != 0.0 && cloud.points[3].x != 0.0)
        {
            pub.publish(output);

            // Output points to the console
            ROS_INFO("Publishing Points:");
            //for (const auto& point : cloud.points) {
            //    ROS_INFO("Point: x=%f, y=%f, z=%f", point.x, point.y, point.z);
            //}
            ROS_INFO("ELBOW Point: x=%f, y=%f, z=%f",cloud.points[1].x, cloud.points[1].y , cloud.points[1].z );
            ROS_INFO("WRIST Point: x=%f, y=%f, z=%f",cloud.points[2].x, cloud.points[2].y , cloud.points[2].z );
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
/*
    std::cout << "Finished body tracking processing!" << std::endl;

    //window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
 */
    return 0;
}

