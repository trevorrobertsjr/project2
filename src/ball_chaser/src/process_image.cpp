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


    // ROS_INFO(img.encoding);
    // ROS_INFO("%1.2f %1.2f %1.2f",(float)img.data[0],(float)img.data[1],(float)img.data[2]);
    // ROS_INFO("Number of Columns:%1.2f Number of Rows:%1.2f Number of Pixels?:%1.2f Array Size:%1.2f",(float)img.width,(float)img.height,(float)img.width*img.height,(float)img.data.size());
    // ROS_INFO("Step:%1.2f",(float)img.step);
    // ROS_INFO("height: %1.2f width: %1.2f",(float)img.height,(float)img.width );
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
    // uint8_t pixels = img.data;
    

    // bool found = false;
    // int found_width = 0;
    // int pixel_count=0;

    for (int i = left_start; i < left_end; i++) {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) {
            is_left = true;
            break;
        }
    }
    for (int i = center_start; i < center_end; i++) {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) {
            is_center = true;
            break;
        }
    }
    for (int i = right_start; i < right_end; i++) {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) {
            is_right = true;
            break;
        }
    }
    
    if (is_left&&is_center&&is_right) {
        ROS_INFO("Hi Ho, Silver!");
    } else if (is_left) {
        ROS_INFO("Left Turn!");
    } else if (is_right) {
        ROS_INFO("Right Turn!");
    } else {
        ROS_INFO("Full Stop!");
    }

    // ROS_INFO("Number of Pixels?:%1.2f",(float)pixel_count);
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