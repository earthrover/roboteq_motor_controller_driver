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
	ros::NodeHandle nh;
	std::string port;
	int32_t baud;
	nh.param<std::string>("port", port, "/dev/ttyACM0");
	nh.param<int32_t>("baud", baud, 115200);
	int serial_trials = 1;
	while(serial_trials <= 20)
	{
		try
		{
			ser.setPort(port);
			ser.setBaudrate(baud);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR("Unable to open port %s", port.c_str());
		}
		if (ser.isOpen())
		{
			ROS_INFO_STREAM("Serial Port initialized\"");
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
}

bool Driver::configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
{
	std::stringstream str;
	if(request.channel)
	{
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
		<< "%\clsav321654987";
	}
	else
	{
		str << "^" << request.userInput << " " << request.value << "_ "
		<< "%\clsav321654987";
	}
	ser.write(str.str());
	ser.flush();
	response.result = str.str();

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

void Driver::run()
{
	std_msgs::String str1;
	ros::NodeHandle nh;
	cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &Driver::cmd_vel_callback, this);
	nh.getParam("frequencyH", frequencyH);

	typedef std::string Key;
	typedef std::string Val;
	std::map<Key, Val> map_sH;
	nh.getParam("queryH", map_sH);

	std::stringstream ss0;
	std::stringstream ss1;
	std::vector<std::string> KH_vector;

	ss0 << "# c_^echof 1_"; //'#c' cleans the buffer (p. 286, v2.1), '^echof 1' disables serial echo (p. 302, v2.1)
	ss1 << "/\"DH?\",\"?\""; //create data stream (p. 287, v2.1)
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

	ser.write(ss0.str());
	ser.write(ss1.str());
	ser.flush();

	read_publisher = nh.advertise<std_msgs::String>("read", 1000);
	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		ros::spinOnce();
		if (ser.available())
		{
			std_msgs::String result;
			result.data = ser.read(ser.available());

			read_publisher.publish(result);
			boost::replace_all(result.data, "\r", "");
			boost::replace_all(result.data, "+", "");
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
		}
	loop_rate.sleep();
	}
}
