#include "mpc_qp_control/mpc_qp_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_qp_control");
  MPCQPControl obj;
  ros::spin();
  return 0;
};
