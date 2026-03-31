/*
* File:          robot_go_forward.h
* Date:          03-20-2026
* Description:   Simple controller to make the robot move forward
* Author:        Rui Martins
* Modifications:
*/

#ifndef ROBOT_GO_FORWARD_H
#define ROBOT_GO_FORWARD_H

// Structs
typedef struct {
    float x;
    float y;
    float z;
} Position;

typedef struct {
    WbDeviceTag left_motor_sensor_tag;
    WbDeviceTag right_motor_sensor_tag;
    double left_sensor_value;
    double right_sensor_value;
} MotorEncoders;


// Debug
void debug_list_devices();

// Utils
float distance(Position component_01, Position component_02);
float clamp_float(float value, float min_val, float max_val);

// Control
void avoid_obstacles(
    float* sensor_distances,
    int num_sensors,
    WbDeviceTag left_motor,
    WbDeviceTag right_motor
);
float controller_pid(
    float error,
    float error_threshold,
    float kp,
    float ki,
    float kd,
    float dt,
    float error_total
);

#endif /* ROBOT_GO_FORWARD_H */


