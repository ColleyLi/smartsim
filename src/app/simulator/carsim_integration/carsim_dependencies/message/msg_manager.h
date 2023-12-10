#include "app/simulator/carsim_integration/carsim_dependencies/message/msg_base.h" // VS types and definitions

// carsim initialize
struct carsim_init
{
  vs_real *initX;
  vs_real *initY;
  vs_real *initZ;
  vs_real *initYaw;
  vs_real *initV;
};

// carsim I/O channels: import
struct carsim_inport
{
  vs_real *acceleration;
  vs_real *steering_angle;
  vs_real *zgnd_l1;
  vs_real *zgnd_l2;
  vs_real *zgnd_r1;
  vs_real *zgnd_r2;
};

// carsim I/O channels: export
struct carsim_outport
{
  vector3d pos;
  vector3d velocity;
  vector3d accel;
  vector3d angular_vel;
  vector3d angular_acc;      /* Angular accelaration [rad/s2] */
  vs_real *roll;
  vs_real *pitch;
  vs_real *yaw;
  vs_real *steering;
  vs_real *CmpS_L1;                /* Compression of ride spring L1 [mm] */
  vs_real *CmpS_L2;                /* Compression of ride spring L2 [mm] */
  vs_real *CmpS_R1;                /* Compression of ride spring R1 [mm] */
  vs_real *CmpS_R2;                /* Compression of ride spring R2 [mm] */
  vs_real *RRE_L1;                 /* Effective rolling rad, tire L1 [mm] */
  vs_real *RRE_L2;                 /* Effective rolling rad, tire L2 [mm] */
  vs_real *RRE_R1;                 /* Effective rolling rad, tire R1 [mm] */
  vs_real *RRE_R2;                 /* Effective rolling rad, tire R2 [mm] */
  vs_real *Rot_L1;                 /* Wheel L1 rotation (Euler) [rev] */
  vs_real *Rot_L2;                 /* Wheel L2 rotation (Euler) [rev] */
  vs_real *Rot_R1;                 /* Wheel R1 rotation (Euler) [rev] */
  vs_real *Rot_R2;                 /* Wheel R2 rotation (Euler) [rev] */
  vs_real *Kappa_L1;               /* Longitudinal slip, tire L1 [-] */
  vs_real *Kappa_L2;               /* Longitudinal slip, tire L2 [-] */
  vs_real *Kappa_R1;               /* Longitudinal slip, tire R1 [-] */
  vs_real *Kappa_R2;               /* Longitudinal slip, tire R2 [-] */
  vs_real *Steer_L1;               /* wheel steer L1 [deg] */
  vs_real *Steer_L2;               /* wheel steer L2 [deg] */
  vs_real *Steer_R1;               /* wheel steer R1 [deg] */
  vs_real *Steer_R2;               /* wheel steer R2 [deg] */
  vs_real *X_L1;                   /* X coordinate, wheel center L1 [m] */
  vs_real *Y_L1;
  vs_real *Z_L1;
  vs_real *X_L2;                   /* X coordinate, wheel center L2 [m] */
  vs_real *Y_L2;
  vs_real *Z_L2;
  vs_real *X_R1;                   /* X coordinate, wheel center R1 [m] */
  vs_real *Y_R1;
  vs_real *Z_R1;
  vs_real *X_R2;                   /* X coordinate, wheel center R2 [m] */
  vs_real *Y_R2;
  vs_real *Z_R2;
  vs_real *Avy_L1;                 /* Driveline wheel L1 spin speed [rpm] */
  vs_real *Avy_L2;                 /* Driveline wheel L2 spin speed [rpm] */
  vs_real *Avy_R1;                 /* Driveline wheel R1 spin speed [rpm] */
  vs_real *Avy_R2;                 /* Driveline wheel R2 spin speed [rpm] */

};



