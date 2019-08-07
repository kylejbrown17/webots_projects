#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
  
  // GPS Sensor
  WbDeviceTag gps = wb_robot_get_device("GPS");
  wb_gps_enable(gps, TIME_STEP);
  
  // IMU sensor
  WbDeviceTag imu = wb_robot_get_device("IMU");
  wb_inertial_unit_enable(imu, TIME_STEP);
  
  // Distance Sensor
  WbDeviceTag ds[2];
  int i;
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  // Actuators (wheels)
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  bool avoid_obstacle_counter = 0;
  // Control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Read sensors
    double ds_values[2];
    for (i = 0; i < 2; i++)
      ds_values[i] = wb_distance_sensor_get_value(ds[i]);
    
    const double* gps_values = wb_gps_get_values(gps);
    const double gps_speed = wb_gps_get_speed(gps);
    printf("GPS readings: position (%f, %f, %f), speed: %f\n",
      gps_values[0],gps_values[1],gps_values[2],
      gps_speed
      );
    
    const double* imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
    printf("IMU readings: roll %f, pitch: %f, yaw: %f\n",
      imu_values[0],imu_values[1],imu_values[2]);
    
    // Process measurements and select actuator commands
    double left_speed = 2.0;
    double right_speed = 2.0;
    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      left_speed = 2.0;
      right_speed = -2.0;
    } else {
      if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
        avoid_obstacle_counter = 20;
    }
    
    // Write commands to actuators
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}