#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver.h>
#include <sstream>

void RoboteqDriver::init()
{
	_nh.param<std::string>("port", _port, "/dev/ttyACM0");
	_nh.param<int32_t>("baud", _baud_rate, 115200);

	connect();
}

void RoboteqDriver::connect()
{
	int serial_trials = 1;
	while(serial_trials <= 20)
	{
		try
		{
			_ser.setPort(port);
			_ser.setBaudrate(baud);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			_ser.setTimeout(to);
			_ser.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR("Unable to open port %s", port.c_str());
		}
		if (_ser.isOpen())
		{
			ROS_INFO("Serial Port %s initialized", port.c_str());
			break;
		}
		else
		{
			serial_trials += 1;
		}
		sleep(1);
	}
}

void RoboteqDriver::cmd_vel_callback(const geometry_msgs::Twist &msg)
{
	std::stringstream cmd_sub;
	cmd_sub << "!G 1" << " " << msg.linear.x << "_"
			    << "!G 2" << " " << msg.angular.z << "_";

	_ser.write(cmd_sub.str());
	_ser.flush();
}

bool RoboteqDriver::configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
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
	_ser.write(str.str());
	_ser.flush();
	response.result = str.str();

	ROS_INFO_STREAM(response.result);
	return true;
}

bool Driver::commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
{
	std::stringstream str;
	str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
	_ser.write(str.str());
	_ser.flush();
	response.result = str.str();

	ROS_INFO_STREAM(response.result);
	return true;
}

bool Driver::maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
{
	std::stringstream str;
	str << "%" << request.userInput << " "
		<< "_";
	_ser.write(str.str());
	_ser.flush();

	response.result = _ser.read(ser.available());

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

	_ser.write(ss0.str());
	_ser.write(ss1.str());
	_ser.flush();

	read_publisher = nh.advertise<std_msgs::String>("read", 1000);
	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		ros::spinOnce();
		if (_ser.available())
		{
			std_msgs::String result;
			result.data = _ser.read(_ser.available());

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
