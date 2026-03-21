/*
* File:          robot_go_forward.c
* Date:          03-20-2026
* Description:   Simple controller to make the robot move forward
* Author:        Rui Martins
* Modifications:
*/

#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/device.h>
#include <webots/motor.h>
#include <webots/lidar.h>

#include "robot_go_forward.h"

// Globals
#define DEBUG 1

#define TIME_STEP 64

#define LIDAR_SAMPLING_PERIOD TIME_STEP
#define LIDAR_SENSORS_NUM 3

#define MOTOR_MAX_SPEED 6.28

#define OBSTACLE_MAX_DIST 0.25

//
int main(int argc, char **argv) {
    /* necessary to initialize webots stuff */
    wb_robot_init();

    // List devices
    if (DEBUG == 1) {debug_list_devices();}

    // Devices
    WbDeviceTag lidar       = wb_robot_get_device("lidar");

    WbDeviceTag left_motor  = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

    // Robot
    Position robot_position = (Position) {
        .x = 0.0,
        .y = 0.0,
        .z = 0.0,
    };

    // Lidar
    wb_lidar_enable(lidar, LIDAR_SAMPLING_PERIOD);
    wb_lidar_enable_point_cloud(lidar);

    int lidar_num_points = wb_lidar_get_horizontal_resolution(lidar);
    const WbLidarPoint* lidar_pcd;
    Position* lidar_pcd_pos = malloc(sizeof(Position) * lidar_num_points);

    // Motors
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    
    wb_motor_set_velocity(left_motor, MOTOR_MAX_SPEED);
    wb_motor_set_velocity(right_motor, MOTOR_MAX_SPEED);
    
    /*
    * main loop
    */
    while (wb_robot_step(TIME_STEP) != -1) {
        // Read sensors
        lidar_pcd =  wb_lidar_get_point_cloud(lidar);
        
        // Process
        for (size_t i = 0; i < lidar_num_points; i++) {
            lidar_pcd_pos[i] = (Position) {
                .x = lidar_pcd[i].x,
                .y = lidar_pcd[i].y,
                .z = lidar_pcd[i].z,
            };
        }
        if (DEBUG == 1) {printf("Lidar point %d: %f\n", 0, distance(robot_position, lidar_pcd_pos[0]) );}
        if (DEBUG == 1) {printf("Lidar point %d: %f\n", 7, distance(robot_position, lidar_pcd_pos[7]) );}
        if (DEBUG == 1) {printf("Lidar point %d: %f\n", 15, distance(robot_position, lidar_pcd_pos[15]) );}
        

        // Actuate
        float sensor_distances[LIDAR_SENSORS_NUM] = {
            distance(robot_position, lidar_pcd_pos[0]),
            distance(robot_position, lidar_pcd_pos[7]),
            distance(robot_position, lidar_pcd_pos[15])
        };
        avoid_obstacles(sensor_distances, LIDAR_SENSORS_NUM, left_motor, right_motor);
        
    };

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}

// Debug
void debug_list_devices() {
    int num_devices = wb_robot_get_number_of_devices();
    printf("Number of devices: %d\n", num_devices);

    for (size_t i = 0; i < num_devices; i++)
    {
        WbDeviceTag current_device      = wb_robot_get_device_by_index(i);
        const char* current_device_name = wb_device_get_name(current_device);

        printf("Device %d name is: %s\n", current_device, current_device_name);
    }
}

// Distance
float distance(Position component_01, Position component_02) {
    float distance = sqrtf(
        powf(component_01.x - component_02.x, 2)
        +
        powf(component_01.y - component_02.y, 2)
        +
        powf(component_01.z - component_02.z, 2)
    );
    return distance;
}

// Path control
void avoid_obstacles(
    float* sensor_distances, int num_sensors,
    WbDeviceTag left_motor, WbDeviceTag right_motor
) {
    if (
        sensor_distances[0] <= OBSTACLE_MAX_DIST/2
        ||
        sensor_distances[1] <= OBSTACLE_MAX_DIST
        ||
        sensor_distances[2] <= OBSTACLE_MAX_DIST/2
    ) {
        if (sensor_distances[0] > sensor_distances[2]) {
            wb_motor_set_velocity(left_motor, -MOTOR_MAX_SPEED);
            wb_motor_set_velocity(right_motor, MOTOR_MAX_SPEED);
        } else {
            wb_motor_set_velocity(left_motor, MOTOR_MAX_SPEED);
            wb_motor_set_velocity(right_motor, -MOTOR_MAX_SPEED);
        }
    } else {
        wb_motor_set_velocity(left_motor, MOTOR_MAX_SPEED);
        wb_motor_set_velocity(right_motor, MOTOR_MAX_SPEED);
    }
}