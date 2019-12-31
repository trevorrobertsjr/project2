#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
        // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot...Danger Will Robinson!");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int height_start = 0;
    int height_end = img.height;
    int left_start = 0;
    int left_end = img.step/3;
    int center_start = left_end;
    int center_end = left_end *2;
    int right_start = center_end;
    int right_end = img.step-1;
    bool is_left = false;
    bool is_center = false;
    bool is_right = false;

// Search in the left third of the screen
    for (int i = left_start; i < left_end; i++) {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) {
            is_left = true;
            ROS_INFO("White Left Found");
            break;
        }
    }
// Search in the center third of the screen
    for (int i = center_start; i < center_end; i++) {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) {
            is_center = true;
            ROS_INFO("White Center Found");
            break;
        }
    }
// Search in the right third of the screen
    for (int i = right_start; i < right_end; i++) {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) {
            is_right = true;
            ROS_INFO("White Right Found");
            break;
        }
    }
    
    if ((is_left&&is_center&&is_right) || (!is_left&&is_center&&!is_right)) {
        ROS_INFO("Drive Forward");
    }
    if (is_left&&!is_right) {
        ROS_INFO("Left Turn!");
    }
    if (is_right&&!is_left) {
        ROS_INFO("Right Turn!");
    }
    if (!is_left&&!is_center&&!is_right) {
        ROS_INFO("Full Stop!");
    }
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