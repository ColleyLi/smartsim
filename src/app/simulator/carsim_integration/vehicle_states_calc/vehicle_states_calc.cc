#include "app/simulator/carsim_integration/vehicle_states_calc/vehicle_states_calc.h"

namespace smartsil::app::carsim_integration {

void external_setdef (void) {
  // set default values for parameters defined in this file
  sUseExternal = 1;

  // Make sure deg/m units are defined
  vs_define_units((char*)"deg/m", 180.0/PI);

  // setup header for Echo file
  vs_add_echo_header("EXTERNAL STEERING MODEL (FROM C++)");

  sTstart = vs_get_var_ptr((char*)"TSTART");
  // define new parameters
  // vs_define_par ("OPT_USE_EXT_PATH_FOLLOWER", &sUseExternal, 1, NULL, 1, 0,
  //   "Use path follower model from external C code?" ); // integer, so NULL units

  // define new outputs using variables that live in this file
  // vs_define_out ("Xpreview", "X coordinate of preview point", &sXprev, "M",
  //   "X coordinate", "Steer control look point", "External steer control");

  //
  // load config
  //
  config_node = YAML::LoadFile(FLAGS_carsim_config);
  control_port = config_node["sil"]["actor_control"]["bind_port"].as<int>();
  ego_vehicle_ip = config_node["sil"]["ego_vehicle"]["remote_ip"].as<std::string>();
  ego_vehicle_port = config_node["sil"]["ego_vehicle"]["remote_port"].as<int>();
  chassis_ip = config_node["sil"]["chassis"]["remote_ip"].as<std::string>();
  chassis_port = config_node["sil"]["chassis"]["remote_port"].as<int>();
  gps_ip = config_node["sil"]["gps"]["remote_ip"].as<std::string>();
  gps_port = config_node["sil"]["gps"]["remote_port"].as<int>();
  imu_ip = config_node["sil"]["imu"]["remote_ip"].as<std::string>();
  imu_port = config_node["sil"]["imu"]["remote_port"].as<int>();

}


void  external_calc (vs_real t, vs_ext_loc where) {
  switch (where) {
    case VS_EXT_AFTER_READ: // after having read the Parsfile inputs
      if (sUseExternal) {
        char vsStateXo[128], vsStateYo[128], vsStateZo[128], vsStateYaw[128], vsStateV[128];
        //
        // input: sim control
        //
        vs_statement("IMPORT", "IMP_AXTARGET replace", 1);
        vs_in.acceleration = vs_get_var_ptr((char*)"IMP_AXTARGET");
        vs_statement("IMPORT", "IMP_STEER_SW replace", 1);
        vs_in.steering_angle = vs_get_var_ptr((char*)"IMP_STEER_SW");

        //
        // input: tire contact ground Z
        //
        vs_in.zgnd_l1 = vs_get_var_ptr((char*)"IMP_ZGND_L1");
        vs_statement("IMPORT", "IMP_ZGND_L1 replace", 1);
        vs_in.zgnd_l2 = vs_get_var_ptr((char*)"IMP_ZGND_L2");
        vs_statement("IMPORT", "IMP_ZGND_L2 replace", 1);
        vs_in.zgnd_r1 = vs_get_var_ptr((char*)"IMP_ZGND_R1");
        vs_statement("IMPORT", "IMP_ZGND_R1 replace", 1);
        vs_in.zgnd_r2 = vs_get_var_ptr((char*)"IMP_ZGND_R2");
        vs_statement("IMPORT", "IMP_ZGND_R2 replace", 1);

        //
        // initParameter: SV_XO, SV_YO, SV_YAW, SV_VXS
        //
        sprintf(vsStateXo, "SV_XO=%.3f", *(vs_init.initX));
        vs_statement("EQ_INIT", vsStateXo, 1);
        sprintf(vsStateYo, "SV_YO=%.3f", *(vs_init.initY));
        vs_statement("EQ_INIT", vsStateYo, 1);
        sprintf(vsStateZo, "SV_ZO=%.3f", *(vs_init.initZ));
        vs_statement("EQ_INIT", vsStateZo, 1);
        sprintf(vsStateYaw, "SV_YAW=%.3f", *(vs_init.initYaw));
        vs_statement("EQ_INIT", vsStateYaw, 1);
        sprintf(vsStateV, "SV_VXS=%.3f", *(vs_init.initV));
        vs_statement("EQ_INIT", vsStateV, 1);

        // output: sim chassis and localization
        // vehicle transform
        vs_out.pos.x = vs_get_var_ptr((char*)"XO");  // inertial, global coord. (m)
        vs_out.pos.y = vs_get_var_ptr((char*)"YO");
        vs_out.pos.z = vs_get_var_ptr((char*)"ZO");
        vs_out.roll  = vs_get_var_ptr((char*)"ROLL_E");   // euler angle, vehicle (rad)
        vs_out.pitch = vs_get_var_ptr((char*)"PITCH");
        vs_out.yaw   = vs_get_var_ptr((char*)"YAW");

        // velocity and angular velocity
        vs_out.velocity.x    = vs_get_var_ptr((char*)"VXNF_SM");  // Long. speed, inst. CG, vehicle (m/s)
        vs_out.velocity.y    = vs_get_var_ptr((char*)"VYNF_SM");
        vs_out.velocity.z    = vs_get_var_ptr((char*)"VZ_SM");
        vs_out.angular_vel.x = vs_get_var_ptr((char*)"AVX"); // Roll rate (body-fixed), vehicle (rad/s)
        vs_out.angular_vel.y = vs_get_var_ptr((char*)"AVY");
        vs_out.angular_vel.z = vs_get_var_ptr((char*)"AVZ");

        // acceleration
        vs_out.accel.x       = vs_get_var_ptr((char*)"AX");  // Long. accel., inst. CG, vehicle (m/s^2)
        vs_out.accel.y       = vs_get_var_ptr((char*)"AY");
        vs_out.accel.z       = vs_get_var_ptr((char*)"AZ");
        vs_out.angular_acc.x = vs_get_var_ptr((char*)"AAX");
        vs_out.angular_acc.y = vs_get_var_ptr((char*)"AAY");
        vs_out.angular_acc.z = vs_get_var_ptr((char*)"AAZ");

        // steering angle
        vs_out.steering = vs_get_var_ptr((char*)"STEER_SW");
        vs_out.Steer_L1 = vs_get_var_ptr((char*)"STEER_L1");
        vs_out.Steer_L2 = vs_get_var_ptr((char*)"STEER_L2");
        vs_out.Steer_R1 = vs_get_var_ptr((char*)"STEER_R1");
        vs_out.Steer_R2 = vs_get_var_ptr((char*)"STEER_R2");

        // wheel
        vs_out.CmpS_L1 = vs_get_var_ptr((char*)"CMPS_L1");
        vs_out.CmpS_L2 = vs_get_var_ptr((char*)"CMPS_L2");
        vs_out.CmpS_R1 = vs_get_var_ptr((char*)"CMPS_R1");
        vs_out.CmpS_R2 = vs_get_var_ptr((char*)"CMPS_R2");
        vs_out.RRE_L1 = vs_get_var_ptr((char*)"RRE_L1");
        vs_out.RRE_L2 = vs_get_var_ptr((char*)"RRE_L2");
        vs_out.RRE_R1 = vs_get_var_ptr((char*)"RRE_R1");
        vs_out.RRE_R2 = vs_get_var_ptr((char*)"RRE_R2");
        vs_out.Rot_L1 = vs_get_var_ptr((char*)"ROT_L1");
        vs_out.Rot_L2 = vs_get_var_ptr((char*)"ROT_L2");
        vs_out.Rot_R1 = vs_get_var_ptr((char*)"ROT_R1");
        vs_out.Rot_R2 = vs_get_var_ptr((char*)"ROT_R2");
        vs_out.Kappa_L1 = vs_get_var_ptr((char*)"KAPPA_L1");
        vs_out.Kappa_L2 = vs_get_var_ptr((char*)"KAPPA_L2");
        vs_out.Kappa_R1 = vs_get_var_ptr((char*)"KAPPA_R1");
        vs_out.Kappa_R2 = vs_get_var_ptr((char*)"KAPPA_R2");
        vs_out.X_L1 = vs_get_var_ptr((char*)"X_L1");
        vs_out.Y_L1 = vs_get_var_ptr((char*)"Y_L1");
        vs_out.Z_L1 = vs_get_var_ptr((char*)"Z_L1");
        vs_out.X_L2 = vs_get_var_ptr((char*)"X_L2");
        vs_out.Y_L2 = vs_get_var_ptr((char*)"Y_L2");
        vs_out.Z_L2 = vs_get_var_ptr((char*)"Z_L2");
        vs_out.X_R1 = vs_get_var_ptr((char*)"X_R1");
        vs_out.Y_R1 = vs_get_var_ptr((char*)"Y_R1");
        vs_out.Z_R1 = vs_get_var_ptr((char*)"Z_R1");
        vs_out.X_R1 = vs_get_var_ptr((char*)"X_R2");
        vs_out.Y_R1 = vs_get_var_ptr((char*)"Y_R2");
        vs_out.Z_R1 = vs_get_var_ptr((char*)"Z_R2");
        vs_out.Avy_L1 = vs_get_var_ptr((char*)"AVY_L1");
        vs_out.Avy_L2 = vs_get_var_ptr((char*)"AVY_L2");
        vs_out.Avy_R1 = vs_get_var_ptr((char*)"AVY_R1");
        vs_out.Avy_R2 = vs_get_var_ptr((char*)"AVY_R2");
        
      }

      break;

    case VS_EXT_EQ_IN: // calculations at the start of a time step
      if (!sUseExternal); // no effect if sUseExternal is FALSE
      else if (t <= *sTstart) {   
        // init vehicle parameters
        *(vs_in.steering_angle) = 0.0;
        *(vs_in.acceleration) = 0.0;
        
      } 
      else { // steer proportional to the lateral error
        using ctlType = vehicle_control::VehicleControl;
        ctlType ctl_result;
        // input: control
        // recv sim control command
        //
        bool success = websocket_control.connectToUdpClient(control_port);
        if (!success) {
          std::cerr << "Failed to connect to udp socket." << std::endl;
        }
        websocket_control.recvData(ctl_result);

        *(vs_in.acceleration) = ctl_result.acceleration;
        *(vs_in.steering_angle)  = ctl_result.steering_angle;
        fprintf(stderr, "accCmd = %.2f\n", *(vs_in.acceleration));
        fprintf(stderr, "strCmd = %.2f\n", *(vs_in.steering_angle));

        websocket_control.closeSocket();

      }
      break;

    case VS_EXT_EQ_OUT: // calculate output variables at the end of a time step
      using chaType = vehicle_state::Chassis;
      using gpsType = vehicle_state::Gps;
      using imuType = vehicle_state::Imu;

      //
      // output: chassis
      //
      websocket_chassis.connectToUdpServer(chassis_ip, chassis_port); // chassis

      chaType chassis;
      chassis.speed          = *(vs_out.velocity.x);
      chassis.acceleration_x = *(vs_out.accel.x);
      chassis.steering_angle = *(vs_out.steering);

      websocket_chassis.sendData(chassis);
      websocket_chassis.closeSocket();

      //
      // output: gps and imu
      //
      websocket_gps.connectToUdpServer(gps_ip, gps_port);  // gps
      websocket_imu.connectToUdpServer(imu_ip, imu_port);  // imu

      // gps
      gpsType gps;
      gps.pos_x = *(vs_out.pos.x);
      gps.pos_y = *(vs_out.pos.y);
      gps.pos_z = *(vs_out.pos.z);
      gps.linear_velocity_vrf_x = *(vs_out.velocity.x);
      gps.linear_velocity_vrf_y = *(vs_out.velocity.y);
      gps.linear_velocity_vrf_z = *(vs_out.velocity.z);

      websocket_gps.sendData(gps);
      websocket_gps.closeSocket();

      // imu
      imuType imu;
      imu.linear_acceleration_vrf_x = *(vs_out.accel.x);
      imu.linear_acceleration_vrf_y = *(vs_out.accel.y);
      imu.linear_acceleration_vrf_z = *(vs_out.accel.z);
      imu.angular_velocity_vrf_x = *(vs_out.angular_vel.x);
      imu.angular_velocity_vrf_y = *(vs_out.angular_vel.y);
      imu.angular_velocity_vrf_z = *(vs_out.angular_vel.z);
      imu.roll = *(vs_out.roll);
      imu.pitch = *(vs_out.pitch);
      imu.yaw = *(vs_out.yaw);

      websocket_imu.sendData(imu);
      websocket_imu.closeSocket();

      break;
  }
}


void external_echo (vs_ext_loc where) {
  static char buffer[200];

  switch (where) {
    case VS_EXT_ECHO_TOP: // top of echo file
      vs_write_to_echo_file (
      "! carsim linux vehicle simulation model with udp socket to provide chassis and localization msg to smartcar\n");
      break;

    case VS_EXT_ECHO_SYPARS: // end of system parameter section
      break;

    case VS_EXT_ECHO_PARS: // end of model parameter section
      break;

    case VS_EXT_ECHO_END: // end of echo file
      break;
  }
}

vs_bool external_scan(char* keyword, char* buffer) {
  // place for code to look at keyword and buffer
  return 0;
}

} // namespace smartsil::app::carsim_integration

