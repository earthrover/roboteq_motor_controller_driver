#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <roboteq_motor_controller_driver/querylist.h>

serial::Serial ser;

using namespace roboteq;

void Driver::connect()
{
	ros::NodeHandle n;
	int serial_trials = 1;

	while(serial_trials <= 20)
	{
		try
		{
			ser.setPort("/dev/ttyACM0");
			ser.setBaudrate(115200);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR_STREAM("Unable to open port ");
		}
		if (ser.isOpen())
		{
			ROS_INFO_STREAM("Serial Port initialized ");
			break;
		}
		else
		{
			serial_trials += 1;
		}
		sleep(1);
	}
}

void Driver::cmd_vel_callback(const geometry_msgs::Twist &msg)
{
	std::stringstream cmd_sub;
	cmd_sub << "!G 1"
			<< " " << msg.linear.x << "_"
			<< "!G 2"
			<< " " << msg.angular.z << "_";

	ser.write(cmd_sub.str());
	ser.flush();
	ROS_INFO_STREAM(cmd_sub.str());
}
void Driver::roboteq_subscriber()
{
	ros::NodeHandle n;
	cmd_vel_sub = n.subscribe("/cmd_vel", 10, &Driver::cmd_vel_callback, this);
}

bool Driver::configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
{
	size_t n = request.channel.size();
	std::stringstream ss;

	for(size_t i{0}; i<n; i++)
	{
		if(request.channel[i])
		{
			ss << "^" << request.userInput[i] << " " << request.channel[i] << " " << request.value[i] << "_";
		}
		else
		{
			ss << "^" << request.userInput[i] << " " << request.value[i]<< "_";
		}
		if (i==n-1)
		{
			ss << " %\clsav321654987";
		}
	}
	ser.write(ss.str());
	ser.flush();
	response.result = ss.str();

	ROS_INFO_STREAM(response.result);
	return true;
}

bool Driver::commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
{
	std::stringstream str;
	str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
	ser.write(str.str());
	ser.flush();
	response.result = str.str();

	ROS_INFO_STREAM(response.result);
	return true;
}

bool Driver::maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
{
	std::stringstream str;
	str << "%" << request.userInput << " "
		<< "_";
	ser.write(str.str());
	ser.flush();

	response.result = ser.read(ser.available());

	ROS_INFO_STREAM(response.result);
	return true;
}

void Driver::roboteq_services()
{
	ros::NodeHandle n;
	configsrv = n.advertiseService("config_service", &Driver::configservice, this);
	commandsrv = n.advertiseService("command_service", &Driver::commandservice, this);
	maintenancesrv = n.advertiseService("maintenance_service", &Driver::maintenanceservice, this);
}

bool Driver::load_manual_profile()
{
	ros::NodeHandle n;
	int mixing_mode, operating_mode, max_power_adjust, cmd_prior_1, cmd_prior_2;
  float proportional_gain, integral_gain;
	float cl_speed_position_KP = 0, cl_speed_KP = 0, cl_speed_position_KI = 0, cl_speed_KI = 0;

	roboteq_motor_controller_driver::config_srv srv;

	n.param<int>("/roboteq_controller_parameters/Manual/command_priority_1", cmd_prior_1, 1);
	n.param<int>("/roboteq_controller_parameters/Manual/command_priority_2", cmd_prior_2, 2);
	n.param<int>("/roboteq_controller_parameters/Manual/mixing_mode", mixing_mode, 1);
	n.param<int>("/roboteq_controller_parameters/Manual/operating_mode", operating_mode, 0);
	n.param<int>("/roboteq_controller_parameters/Manual/max_power_adjust", max_power_adjust, 25);
	n.param<float>("/roboteq_controller_parameters/Manual/KP", proportional_gain, 0);
	n.param<float>("/roboteq_controller_parameters/Manual/KI", integral_gain, 0);

	std::vector<std::string> usr_input{"CPRI", "CPRI", "MXMD", "MMOD", "MMOD", "MXPF", "MXPF", "MXPR", "MXPR","KPG", "KPG", "KIG", "KIG", "KPG", "KPG", "KIG", "KIG"};
  std::vector<int64_t> val{cmd_prior_1, cmd_prior_2, mixing_mode, operating_mode, operating_mode, max_power_adjust, max_power_adjust, max_power_adjust, max_power_adjust, cl_speed_KP, cl_speed_KP, cl_speed_KI, cl_speed_KI, cl_speed_position_KP, cl_speed_position_KP, cl_speed_position_KI, cl_speed_position_KI};
  std::vector<int64_t> ch{1, 2, 0, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 3, 4, 3, 4};

	srv.request.value = val;
  srv.request.userInput = usr_input;
  srv.request.channel = ch;

	return configservice(srv.request, srv.response);
}

void Driver::run()
{

	std_msgs::String str1;
	ros::NodeHandle nh;
	nh.getParam("frequencyH", frequencyH);
	nh.getParam("frequencyL", frequencyL);
	nh.getParam("frequencyG", frequencyG);

	typedef std::string Key;
	typedef std::string Val;
	std::map<Key, Val> map_sH;
	nh.getParam("queryH", map_sH);

	std::stringstream ss0;
	std::stringstream ss1;
	std::stringstream ss2;
	std::stringstream ss3;
	std::vector<std::string> KH_vector;

	ss0 << "^echof 1_";
	ss1 << "# c_/\"DH?\",\"?\"";
	for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
	{
		Key KH = iter->first;

		KH_vector.push_back(KH);

		Val VH = iter->second;
		ss1 << VH << "_";
	}
	ss1 << "# " << frequencyH << "_";

	std::vector<ros::Publisher> publisherVecH;
	for (int i = 0; i < KH_vector.size(); i++)
	{
		publisherVecH.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KH_vector[i], 100));
	}

	std::map<Key, Val> map_sL;
	nh.getParam("queryL", map_sL);
	std::vector<std::string> KL_vector;
	ss2 << "/\"DL?\",\"?\"";
	for (std::map<Key, Val>::iterator iter = map_sL.begin(); iter != map_sL.end(); ++iter)
	{
		Key KL = iter->first;
		KL_vector.push_back(KL);

		Val VL = iter->second;
		ss2 << VL << "_";
	}
	ss2 << "# " << frequencyL << "_";

	std::vector<ros::Publisher> publisherVecL;
	for (int i = 0; i < KL_vector.size(); ++i)
	{
		publisherVecL.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KL_vector[i], 100));
	}

	std::map<Key, Val> map_sG;
	nh.getParam("queryG", map_sG);
	std::vector<std::string> KG_vector;
	ss3 << "/\"DG?\",\"?\"";
	for (std::map<Key, Val>::iterator iter = map_sG.begin(); iter != map_sG.end(); ++iter)
	{
		Key KG = iter->first;
		KG_vector.push_back(KG);

		Val VG = iter->second;
		ss3 << VG << "_";
	}
	ss3 << "# " << frequencyG << "_";

	std::vector<ros::Publisher> publisherVecG;
	for (int i = 0; i < KG_vector.size(); ++i)
	{
		publisherVecG.push_back(nh.advertise<std_msgs::String>(KG_vector[i], 100));
	}

	ser.write(ss0.str());
	ser.write(ss1.str());
	ser.write(ss2.str());
	ser.write(ss3.str());
	ser.flush();

	int init_counter = 1;
	read_publisher = nh.advertise<std_msgs::String>("read", 1000);
	ros::Rate loop_rate(5);

	while (ros::ok())
	{

		ros::spinOnce();
		if (ser.available())
		{

			//ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String result;
			result.data = ser.read(ser.available());

			read_publisher.publish(result);
			boost::replace_all(result.data, "\r", "");
			boost::replace_all(result.data, "+", "");
			if(init_counter < 5)
			{
				init_counter += 1;
			}
			else
			{
				std::vector<std::string> fields;
				boost::split(fields, result.data, boost::algorithm::is_any_of("D"));
				std::vector<std::string> fields_H;
				boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));
				if (fields_H[0] == "H")
				{

					for (int i = 0; i < publisherVecH.size(); ++i)
					{

						std::vector<std::string> sub_fields_H;

						boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
						roboteq_motor_controller_driver::channel_values Q1;

						Q1.value.push_back(0);
						for (int j = 0; j < sub_fields_H.size(); j++)
						{
							Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
						}

						publisherVecH[i].publish(Q1);
					}
				}
				else
				{
					ROS_INFO_STREAM("Garbage data on Serial");
				}

				std::vector<std::string> fields_L;
				std::vector<std::string> fields_G;
				boost::split(fields_G, fields[3], boost::algorithm::is_any_of("?"));
				boost::split(fields_L, fields[2], boost::algorithm::is_any_of("?"));
				if (fields_L[0] == "L")
				{

					for (int i = 0; i < publisherVecL.size(); ++i)
					{
						std::vector<std::string> sub_fields_L;

						boost::split(sub_fields_L, fields_L[i + 1], boost::algorithm::is_any_of(":"));

						roboteq_motor_controller_driver::channel_values Q1;
						Q1.value.push_back(0);
						for (int j = 0; j < sub_fields_L.size(); j++)
						{

							Q1.value.push_back(boost::lexical_cast<int>(sub_fields_L[j]));
						}

						publisherVecL[i].publish(Q1);
					}
				}

				if (fields_G[0] == "G")
				{

					for (int i = 0; i < publisherVecG.size(); ++i)
					{
						std_msgs::String Q1;
						Q1.data = fields_G[i + 1];

						publisherVecG[i].publish(Q1);
					}
				}

				//ROS_INFO_STREAM("success!");

				Driver::roboteq_subscriber();
			}
		}
		loop_rate.sleep();
		//ROS_INFO_STREAM("Type the command - \"rostopic list\" - in new terminal for publishers");
	}
}
