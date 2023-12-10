#pragma once

typedef struct {
    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;
    double accel_x;
    double accel_y;
    double accel_z;
    double angular_vel_x;
    double angular_vel_y;
    double angular_vel_z;
    double steer_angle;
    double roll;
    double pitch;
    double yaw;
}VehilceLocalization;