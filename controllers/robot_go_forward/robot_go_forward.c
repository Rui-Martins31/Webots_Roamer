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
#define TRUE 1
#define FALSE 0

#define DEBUG TRUE

// Time step
#define TIME_STEP 64

// Lidar
#define LIDAR_SAMPLING_PERIOD TIME_STEP
#define LIDAR_SENSORS_NUM 3

// PID
#define CONTROLLER_KP 1.0
#define CONTROLLER_KI 1.0
#define CONTROLLER_KD 1.0
#define CONTROLLER_ERROR_THRESHOLD 0.1

#define MOTOR_MAX_SPEED 6.28 * 0.999

#define OBSTACLE_MAX_DIST 0.25

//
int main(int argc, char **argv) {
    /* necessary to initialize webots stuff */
    wb_robot_init();

    // List devices
    if (DEBUG == TRUE) {debug_list_devices();}

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
        if (DEBUG == TRUE) {printf("Lidar point %d: %f\n", 0, distance(robot_position, lidar_pcd_pos[0]) );}
        if (DEBUG == TRUE) {printf("Lidar point %d: %f\n", 7, distance(robot_position, lidar_pcd_pos[7]) );}
        if (DEBUG == TRUE) {printf("Lidar point %d: %f\n", 15, distance(robot_position, lidar_pcd_pos[15]) );}
        

        // Actuate
        float sensor_distances[LIDAR_SENSORS_NUM] = {
            distance(robot_position, lidar_pcd_pos[0]),
            distance(robot_position, lidar_pcd_pos[7]),
            distance(robot_position, lidar_pcd_pos[15])
        };
        avoid_obstacles(sensor_distances, LIDAR_SENSORS_NUM, left_motor, right_motor);

        // DEBUG
        printf("\n");
        
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

// Utils
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

float clamp_float(float value, float min_val, float max_val) {
    return fmaxf(min_val, fminf(value, max_val));
}

// Path control
void avoid_obstacles(
    float* sensor_distances, int num_sensors,
    WbDeviceTag left_motor, WbDeviceTag right_motor
) {
    // if (
    //     sensor_distances[0] <= OBSTACLE_MAX_DIST/2
    //     ||
    //     sensor_distances[1] <= OBSTACLE_MAX_DIST
    //     ||
    //     sensor_distances[2] <= OBSTACLE_MAX_DIST/2
    // ) {
    //     if (sensor_distances[0] > sensor_distances[2]) {
    //         wb_motor_set_velocity(left_motor, -MOTOR_MAX_SPEED);
    //         wb_motor_set_velocity(right_motor, MOTOR_MAX_SPEED);
    //     } else {
    //         wb_motor_set_velocity(left_motor, MOTOR_MAX_SPEED);
    //         wb_motor_set_velocity(right_motor, -MOTOR_MAX_SPEED);
    //     }
    // } else {
    //     wb_motor_set_velocity(left_motor, MOTOR_MAX_SPEED);
    //     wb_motor_set_velocity(right_motor, MOTOR_MAX_SPEED);
    // }

    // Check for Nan
    for (u_int8_t i = 0; i < num_sensors; i++) {
        if (isnan(sensor_distances[i])) {
            return;
        }
    }

    // Compute error
    float error = sensor_distances[1] - OBSTACLE_MAX_DIST;

    float motor_speed_factor = controller_pid(
        error,
        CONTROLLER_ERROR_THRESHOLD,
        CONTROLLER_KP,
        CONTROLLER_KI,
        CONTROLLER_KD,
        TIME_STEP
    );
    printf("Motor speed factor: %f\n", motor_speed_factor);

    // Motor control
    wb_motor_set_velocity(
        left_motor,
        clamp_float(/*MOTOR_MAX_SPEED **/ motor_speed_factor, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED)
    );
    wb_motor_set_velocity(
        right_motor,
        clamp_float(/*MOTOR_MAX_SPEED **/ motor_speed_factor, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED)
    );
}

float controller_pid(
    float error,
    float error_threshold,
    float kp,
    float ki,
    float kd,
    float dt
) {
    if (error <= error_threshold) { error = 0.0; }

    float motor_speed_factor = 
        error * kp +
        error * dt * kd;

    return motor_speed_factor;
}