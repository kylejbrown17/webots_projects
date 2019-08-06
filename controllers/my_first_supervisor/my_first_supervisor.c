#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

int main() {
  wb_robot_init();

  // Get robot node references
  char robot_names[2][10] = {"ROBOT_1", "ROBOT_2"};
  WbNodeRef robot_nodes[2]; // references to robot nodes
  WbFieldRef trans_fields[2]; // references to "translation" field of each robot
  int id[2];
  int i;
  for (i = 0; i < 2; i++){
    robot_nodes[i] = wb_supervisor_node_get_from_def(robot_names[i]);
    trans_fields[i] = wb_supervisor_node_get_field(robot_nodes[i], "translation");
    id[i] = wb_supervisor_node_get_id(robot_nodes[i]);
  } 
  
  // Move ROBOT_2
  const double INITIAL[3] = { 0, 0.04, 0.1 };
  wb_supervisor_field_set_sf_vec3f(trans_fields[1], INITIAL);
  
  // Print out the positions of both robots repeatedly
  while (wb_robot_step(32) != -1) {
    // this is done repeatedly
    for (i = 0; i < 2; i++){
      const double *trans = wb_supervisor_field_get_sf_vec3f(trans_fields[i]);
      printf("Robot with id %i is at position: %g %g %g\n", id[i], trans[0], trans[1], trans[2]);
    }
    printf("\n");
  }

  wb_robot_cleanup();

  return 0;
}
