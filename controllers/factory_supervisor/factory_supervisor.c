#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <webots/connector.h>

#include <julia.h>

int main() {

  jl_init();
  (void)jl_eval_string("println(sqrt(2.0))");
  
  printf("Hello from the supervisor!\n");
  
  wb_robot_init();

  // Get robot node references
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT1");
  // int id = wb_supervisor_node_get_id(robot_node);
  // WbNodeRef connector_node = wb_supervisor_node_get_from_def("BLOCK1_CONNECTOR");
  // int connector_id = wb_supervisor_node_get_id(connector_node);
  // WbFieldRef pos1            = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef controller_args = wb_supervisor_node_get_field(robot_node, "controllerArgs");
  wb_supervisor_field_set_sf_string(controller_args, "2.5 1.5 6.0 0.5 2.0 [EAST,EAST,EAST,EAST,NORTH]");
  wb_supervisor_node_restart_controller(robot_node);
  // WbFieldRef autoLock = wb_supervisor_node_get_field(connector_node, "autoLock");
  // wb_supervisor_field_set_sf_bool(isLocked, 0);
  // wb_supervisor_field_set_sf_bool(autoLock, 0);
  // store the last time a message was displayed
  double display_counter = 0.0;
  double display_rate = 1.0; // print sensor and other information every `display_rate` seconds
  
  while (wb_robot_step(32) != -1) {
  
    if (wb_robot_get_time() - display_counter > display_rate) {
      display_counter = wb_robot_get_time();
      printf("t: %f\n", wb_robot_get_time());
      
      printf("controller args: %s\n", wb_supervisor_field_get_sf_string(controller_args));
      
    }
  }

  wb_robot_cleanup();
  jl_atexit_hook(0);

  return 0;
}
