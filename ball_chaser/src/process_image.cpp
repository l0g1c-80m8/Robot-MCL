#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float linear_x, float angular_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Moving the robot towards the ball");

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    float linear_max_speed = 0.4;
    float angular_max_speed = 0.5;
    float window_size = img.step / 3;

    int white_pose_start = -1, white_pose_end = -1;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < img.height * img.step; i++) {
        // img.height -> rows in the matrix, img.step -> cols in the matrix
        if (img.data[i] == white_pixel) {
            if (white_pose_start == -1) white_pose_start = i;
            else if(white_pose_start > -1) white_pose_end = i;
        }
    }

    if (white_pose_start == -1 || white_pose_end == -1) {
        drive_robot(0.0, 0.0);
        return;
    } else {
        float ball_mid_col = ((white_pose_start % img.step) + (white_pose_end % img.step)) / 2;
        if (ball_mid_col < window_size)
            drive_robot(0.0, angular_max_speed);
        else if (ball_mid_col >= window_size && ball_mid_col <= 2 * window_size)
            drive_robot(linear_max_speed, 0.0);
        else
            drive_robot(0.0, -angular_max_speed);
    }
    ROS_INFO_STREAM("Processed image and sent speed set request ");
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}