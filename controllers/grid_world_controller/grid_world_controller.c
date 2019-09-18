/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/motor.h>
#include <webots/robot.h>

#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

#include <stdio.h>
#include <stdlib.h>

#include <julia.h>

// Khepera IV Robot parameters
#define MAX_SPEED 47.6
#define TRACKWIDTH 0.1054

// arena dimensions and resolution
#define CELL_WIDTH 0.5
#define X_OFFSET 0.0
#define Y_OFFSET 0.0
#define THETA_OFFSET 3.141592653589

// #define NUMBER_OF_ULTRASONIC_SENSORS 5
// static const char *ultrasonic_sensors_names[NUMBER_OF_ULTRASONIC_SENSORS] = {
//   "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
//   "right ultrasonic sensor"};
//
// #define NUMBER_OF_INFRARED_SENSORS 12
// static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {
//   // turret sensors
//   "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
//   "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
//   // ground sensors
//   "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
//   "ground right infrared sensor"};

int main(int argc, char **argv) {
  printf("num args %i\n", argc);
  char* robot_name;
  char* x0;
  char* y0;
  char* t0;
  char* cell_width;
  char* transition_time;
  char* instructions;
  if (argc == 8){
    robot_name      = argv[1];
    x0              = argv[2];
    y0              = argv[3];
    t0              = argv[4];
    cell_width      = argv[5];
    transition_time = argv[6];
    instructions    = argv[7];
  } else {
    robot_name      = "Default Robot";
    x0              = "2.5";
    y0              = "1.5";
    t0              = "4.0";
    cell_width      = "0.5";
    transition_time = "2.0";
    instructions    = "[EAST,NORTH,EAST,SOUTH,EAST,NORTH,NORTH,WEST]";
  };
  
  printf("Hello from %s\n", robot_name);
  char argbuffer[10000];
  (void)sprintf(argbuffer,
    "grid_path = construct_grid_world_path(%s,%s,%s,%s,%s,%s)",
    x0, y0, t0, cell_width, transition_time, instructions);
  printf("%s\n",argbuffer);
  /*
  Julia stuff
  */
  jl_init();
  jl_value_t* refs = jl_eval_string("refs = IdDict()");
  jl_function_t* setindex = jl_get_function(jl_main_module, "setindex!");
  
  jl_eval_string("using Pkg");
  jl_eval_string("Pkg.activate(joinpath(Pkg.devdir(),\"WebotsSim\"))");
  jl_eval_string("using Vec");
  jl_eval_string("using GridWorldPathFollowing");
  jl_eval_string("GridWorldPathFollowing.warmup()");
  // get required modules and functions
  jl_module_t *GridWorldPathFollowing = (jl_module_t *)jl_eval_string("GridWorldPathFollowing");
  jl_function_t *construct_trajectory = jl_get_function(GridWorldPathFollowing, "construct_trajectory");
  jl_function_t *UnicycleController = jl_get_function(GridWorldPathFollowing,"UnicycleController");
  jl_function_t *optimize_velocity_profile_traj_only = jl_get_function(GridWorldPathFollowing, "optimize_velocity_profile_traj_only");
  jl_function_t *get_action = jl_get_function(GridWorldPathFollowing, "get_action");
  jl_function_t *get_state = jl_get_function(GridWorldPathFollowing, "get_state");
  // initialize the switching_controller and robot model
  jl_value_t *switching_controller  = jl_eval_string("switching_controller = SwitchingController()");
  jl_value_t *model                 = jl_eval_string("model = UnicycleModel()");

  if (jl_exception_occurred())
    printf("Exception occured: %s \n", jl_typeof_str(jl_exception_occurred()));

  // Get the optimized trajectory

  // int n=sprintf(argbuffer,
  //   "grid_path = construct_grid_world_path(%f,%f,%f,%f,%f,%s)",
  //   2.5, 1.5, 4.0, 0.5, 2.0, "[EAST,NORTH,EAST,SOUTH,EAST,NORTH,NORTH,WEST]");
  jl_value_t *grid_path   = jl_eval_string(argbuffer);
  // jl_value_t *grid_path   = jl_eval_string("grid_path = construct_grid_world_path(VecE2(2.5,1.5),4.0,[EAST,NORTH,EAST,SOUTH,EAST,NORTH,NORTH,WEST],0.5,2.0)");
  if (jl_exception_occurred())
    printf("Exception occured: %s \n", jl_typeof_str(jl_exception_occurred()));
  jl_value_t *base_traj   = jl_call1(construct_trajectory, grid_path);
  jl_value_t *traj        = jl_call1(optimize_velocity_profile_traj_only, base_traj);
  jl_call3(setindex, refs, traj, traj);
  // wrap the controller and trajectory into a UnicycleController
  jl_value_t *controller  = jl_call3(UnicycleController,model,traj,switching_controller);
  jl_call3(setindex, refs, controller, controller);
  // state vector type
  jl_value_t *state_type  = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
  jl_call3(setindex, refs, state_type, state_type);

  if (jl_exception_occurred())
      printf("Exception occured: %s \n", jl_typeof_str(jl_exception_occurred()));

  /*
  Webots stuff
  */
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();
  // int i;
  // GPS Sensor
  WbDeviceTag gps = wb_robot_get_device("GPS");
  wb_gps_enable(gps, time_step);
  // IMU sensor
  WbDeviceTag imu = wb_robot_get_device("IMU");
  wb_inertial_unit_enable(imu, time_step);

  // // get and enable the camera
  // WbDeviceTag camera = wb_robot_get_device("camera");
  // wb_camera_enable(camera, time_step);
  // // get and enable the ultrasonic sensors
  // WbDeviceTag ultrasonic_sensors[5];
  // for (i = 0; i < 5; ++i) {
  //   ultrasonic_sensors[i] = wb_robot_get_device(ultrasonic_sensors_names[i]);
  //   wb_distance_sensor_enable(ultrasonic_sensors[i], time_step);
  // }
  // // get and enable the infrared sensors
  // WbDeviceTag infrared_sensors[12];
  // for (i = 0; i < 12; ++i) {
  //   infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
  //   wb_distance_sensor_enable(infrared_sensors[i], time_step);
  // }
  // // get the led actuators
  // WbDeviceTag leds[3] = {wb_robot_get_device("front left led"), wb_robot_get_device("front right led"),
  //                        wb_robot_get_device("rear led")};

  // get the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor, right_motor;
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // store the last time a message was displayed
  double display_counter = -1.0;
  double display_rate = 1.0; // print sensor and other information every `display_rate` seconds

  // main loop
  while (wb_robot_step(time_step) != -1) {

    const double* gps_values = wb_gps_get_values(gps);
    // const double gps_speed = wb_gps_get_speed(gps);
    const double* imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
    // extract state from (noiseless) sensor measurements
    double state[] = {
      gps_values[2] + X_OFFSET,
      gps_values[0] + Y_OFFSET,
      imu_values[2] + THETA_OFFSET
      };

    // get controller command
    jl_array_t *state_vec = jl_ptr_to_array_1d(state_type, state, 3, 0);
    jl_value_t *t = jl_box_float64(wb_robot_get_time());
    if (jl_exception_occurred())
      printf("Exception occured: %s \n", jl_typeof_str(jl_exception_occurred()));

    jl_value_t *target = jl_call3(get_state,model,traj,t);
    if (jl_exception_occurred())
      printf("Exception occured in get_state: %s \n", jl_typeof_str(jl_exception_occurred()));
    jl_array_t *target_array = (jl_array_t*)target;
    double *target_data = (double*)jl_array_data(target_array);

    jl_value_t *wheel_speed_cmd = jl_call3(get_action,controller,(jl_value_t*)state_vec,t);
    if (jl_exception_occurred()){
      printf("Exception occured in get_action: %s \n", jl_typeof_str(jl_exception_occurred()));
      printf("Robot %s\n",robot_name);
      printf("t: %f\n", wb_robot_get_time());
      printf("gps_values: x=%f, y=%f, z=%f\n", gps_values[0], gps_values[1], gps_values[2]);
      printf("imu_values: pitch=%f, roll=%f, yaw=%f\n", imu_values[0], imu_values[1], imu_values[2]);
      printf("target: x=%f, y=%f, theta=%f\n", target_data[0], target_data[1], target_data[2]);
      printf("state: x=%f, y=%f, theta=%f\n", state[0], state[1], state[2]);
      // printf("cmd: v_left=%f, v_right=%f\n", wheel_speed_cmd_data[0], wheel_speed_cmd_data[1]);
    }
    jl_array_t *wheel_speed_cmd_array = (jl_array_t*)wheel_speed_cmd;
    double *wheel_speed_cmd_data = (double*)jl_array_data(wheel_speed_cmd_array);

      // display some sensor data every second
      // and change randomly the led colors
      // if (wb_robot_get_time() - display_counter > display_rate) {
        // display_counter = wb_robot_get_time();
        // printf("Robot %s\n",robot_name);
        // printf("t: %f\n", wb_robot_get_time());
        // printf("gps_values: x=%f, y=%f, z=%f\n", gps_values[0], gps_values[1], gps_values[2]);
        // printf("imu_values: pitch=%f, roll=%f, yaw=%f\n", imu_values[0], imu_values[1], imu_values[2]);
        // printf("target: x=%f, y=%f, theta=%f\n", target_data[0], target_data[1], target_data[2]);
        // printf("state: x=%f, y=%f, theta=%f\n", state[0], state[1], state[2]);
        // printf("cmd: v_left=%f, v_right=%f\n", wheel_speed_cmd_data[0], wheel_speed_cmd_data[1]);
      // }
    wb_motor_set_velocity(left_motor, wheel_speed_cmd_data[0]);
    wb_motor_set_velocity(right_motor, wheel_speed_cmd_data[1]);
  };

  // webots cleanup
  wb_robot_cleanup();
  // julia cleanup
  jl_atexit_hook(0);

  return EXIT_SUCCESS;
}
