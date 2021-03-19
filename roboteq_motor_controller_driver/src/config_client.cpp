#include "ros/ros.h"
#include <roboteq_motor_controller_driver/config_srv.h>

int main(int argc, char **argv)
 {
   ros::init(argc, argv, "config_client");
   ros::NodeHandle nh;
   ros::ServiceClient client = nh.serviceClient<roboteq_motor_controller_driver::config_srv>("config_service");
   roboteq_motor_controller_driver::config_srv srv;
   std::vector<std::string> usr_input{argv[1]};
   std::vector<int64_t> ch{atoll(argv[2])};
   std::vector<int64_t> val{atoll(argv[3])};
   srv.request.userInput = usr_input;
   srv.request.channel = ch;
   srv.request.value = val;
   if (client.call(srv))
   {
     ROS_INFO("success!");
   }
   else
   {
     ROS_ERROR("Failed to call service");
     return 1;
   }

   return 0;
 }
