#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <webots/connector.h>

int main() {
  wb_robot_init();

  // Get robot node references
  // WbNodeRef robot_node = wb_supervisor_node_get_from_def("UR3e");
  // int id = wb_supervisor_node_get_id(robot_node);
  WbNodeRef connector_node = wb_supervisor_node_get_from_def("BLOCK1_CONNECTOR");
  // int connector_id = wb_supervisor_node_get_id(connector_node);
  WbFieldRef isLocked = wb_supervisor_node_get_field(connector_node, "isLocked");
  WbFieldRef autoLock = wb_supervisor_node_get_field(connector_node, "autoLock");
  // wb_supervisor_field_set_sf_bool(isLocked, 0);
  // wb_supervisor_field_set_sf_bool(autoLock, 0);
  int count = 20;
  while (wb_robot_step(32) != -1) {
    if (count < 0){
      printf("isLocked: %i\n",wb_supervisor_field_get_sf_bool(isLocked));
      printf("autoLock: %i\n",wb_supervisor_field_get_sf_bool(autoLock));
      count = 20;
    }
    count--;
  }

  wb_robot_cleanup();

  return 0;
}
