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

// Debug
void debug_list_devices();

float distance(Position component_01, Position component_02);

void avoid_obstacles(
    float* sensor_distances, int num_sensors,
    WbDeviceTag left_motor, WbDeviceTag right_motor
);

#endif /* ROBOT_GO_FORWARD_H */


