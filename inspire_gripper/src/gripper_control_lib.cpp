#ifndef GRIPPER_CONTROL_LIB_CPP
#define GRIPPER_CONTROL_LIB_CPP

#include <gripper_control.h>
#include <string>

//const double inspire_gripper::gripper_serial::WAIT_FOR_RESPONSE_INTERVAL = 0.5;
//const double inspire_gripper::gripper_serial::INPUT_BUFFER_SIZE = 64;
namespace inspire_gripper
{

	gripper_serial::gripper_serial(ros::NodeHandle *nh) :
		act_position_(-1),
		gripper_state_(0xff),
		com_port_(nullptr)
	{
		//Read launch file params
		nh->param<std::string>("inspire_gripper/portname", port_name_, "/dev/ttyUSB0");
		nh->param<int>("inspire_gripper/baudrate", baudrate_, 115200);
		nh->param<int>("inspire_gripper/gripper_id", gripper_id_, 1);
		nh->param<int>("inspire_gripper/test_flags", test_flags, 0);

		//Initialize and open serial port
		try
		{
			com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));
		}
		catch (const serial::IOException &e)
		{
			ROS_FATAL_STREAM("Gripper: failed to open serial port " << port_name_ << ": " << e.what());
			ROS_FATAL("Gripper: check that the device exists and that the user has permission, e.g. dialout group.");
			ros::shutdown();
			return;
		}
		catch (const serial::SerialException &e)
		{
			ROS_FATAL_STREAM("Gripper: serial exception on " << port_name_ << ": " << e.what());
			ros::shutdown();
			return;
		}
		catch (const std::exception &e)
		{
			ROS_FATAL_STREAM("Gripper: serial initialization failed: " << e.what());
			ros::shutdown();
			return;
		}

		if (com_port_->isOpen())
		{
			ROS_INFO_STREAM("Gripper: Serial port " << port_name_ << " openned");
			int id_state = 0;
			while (1)
			{
				id_state = start(com_port_);
				if (id_state == 1)
					break;
				gripper_id_++;
				if (gripper_id_ > 254)
				{
					ROS_INFO("Id error!!!");
					gripper_id_ = 1;
				}
			}

			//Get initial state and discard input buffer
			while (gripper_state_ == 0xff)
			{
				gripper_state_ = getState(com_port_);
                                gripper_state_ = 0;
				ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
			}

		}
		else
		{
			ROS_ERROR_STREAM("Gripper: Serial port " << port_name_ << " not opened");
			ros::shutdown();
		}
	}

	gripper_serial::~gripper_serial()
	{
		if (com_port_ != nullptr)
		{
			if (com_port_->isOpen())
				com_port_->close();      //Close port
			delete com_port_;        //delete object
		}
	}

	//循环扫描ID,确定ID
	int gripper_serial::start(serial::Serial *port)
	{

		std::vector<uint8_t> output;

		output.push_back(0xEB);
		output.push_back(0x90);
		output.push_back(gripper_id_);
		output.push_back(1);
		output.push_back(0x41);

		unsigned int check_num = 0;

		int len = output[3] + 5;
		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];

		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);

		//Send message to the module and wait for response
		port->write(output);

		ros::Duration(0.015).sleep();

		//Read response
		std::vector<uint8_t> input;
		port->read(input, (size_t)64);
		//ROS_INFO("ok");
		if (input.empty())
			return 0;
		else
			return 1;
	}

	//设置开口限位
	bool gripper_serial::setOpenLimit(serial::Serial *port, int openmax, int openmin)
	{
		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length   
		output.push_back(0x05);
		//Command get state
		output.push_back(0x12);

		unsigned int temp_int1, temp_int2;
		temp_int1 = (unsigned int)openmax;
		temp_int2 = (unsigned int)openmin;

		output.push_back(temp_int1 & 0xff);
		output.push_back((temp_int1 >> 8) & 0xff);
		output.push_back(temp_int2 & 0xff);
		output.push_back((temp_int2 >> 8) & 0xff);
		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;

		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];

		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);

		//Send message to the module
		port->write(output);

		ros::Duration(0.015).sleep();

		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		std::vector<uint8_t> input;

		while (input.empty()) {
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//设置ID
	bool gripper_serial::setID(serial::Serial *port, int id)
	{

		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length  
		output.push_back(2);
		//Command get state
		output.push_back(0x04);

		unsigned int temp_int2;
		temp_int2 = (unsigned int)id;
		gripper_id_ = id;
		output.push_back(temp_int2);

		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;

		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];

		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);

		//Send message to the module
		port->write(output);

		ros::Duration(0.015).sleep();

		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);

		std::vector<uint8_t> input;
		while (input.empty()) {
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//运动目标
	bool gripper_serial::moveTgt(serial::Serial *port, int movetgt)
	{

		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length   
		output.push_back(0x03);
		//Command get state
		output.push_back(0x54);

		unsigned int temp_int1;
		temp_int1 = (unsigned int)movetgt;

		output.push_back(temp_int1 & 0xff);
		output.push_back((temp_int1 >> 8) & 0xff);
		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;


		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];


		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);


		//Send message to the module
		port->write(output);


		ros::Duration(0.015).sleep();


		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		std::vector<uint8_t> input;


		while (input.empty()) {
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];

                //ROS_INFO_STREAM("Write: " << temp[0]);

		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//运动松开
	bool gripper_serial::moveMax(serial::Serial *port, int speed)
	{
		std::vector<uint8_t> output;


		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length   
		output.push_back(0x03);
		//Command get state
		output.push_back(0x11);


		unsigned int temp_int1;

		temp_int1 = (unsigned int)speed;


	        output.push_back(temp_int1 & 0xff);
		output.push_back((temp_int1 >> 8) & 0xff);
		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;


		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];


		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);


		//Send message to the module
		port->write(output);


		ros::Duration(0.015).sleep();


		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		std::vector<uint8_t> input;


		const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(0.5);
        while (ros::ok() && input.size() < 7 && ros::WallTime::now() < deadline)
        {
            std::vector<uint8_t> part;
            port->read(part, (size_t)64);
            input.insert(input.end(), part.begin(), part.end());
        }
        if (input.size() < 7)
            return false;


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//运动抓取
	bool gripper_serial::moveMin(serial::Serial *port, int speed, int power)
	{
		std::vector<uint8_t> output;


		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length   
		output.push_back(0x05);
		//Command get state
		output.push_back(0x10);


		unsigned int temp_int1, temp_int2;




		temp_int1 = (unsigned int)speed;
		temp_int2 = (unsigned int)power;

		output.push_back(temp_int1 & 0xff);
		output.push_back((temp_int1 >> 8) & 0xff);
		output.push_back(temp_int2 & 0xff);
		output.push_back((temp_int2 >> 8) & 0xff);

		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;


		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];


		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);


		//Send message to the module
		port->write(output);


		ros::Duration(0.015).sleep();


		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		std::vector<uint8_t> input;


		const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(0.5);
        while (ros::ok() && input.size() < 7 && ros::WallTime::now() < deadline)
        {
            std::vector<uint8_t> part;
            port->read(part, (size_t)64);
            input.insert(input.end(), part.begin(), part.end());
        }
        if (input.size() < 7)
            return false;


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//运动持续抓取
	bool gripper_serial::moveMinHold(serial::Serial *port, int speed, int power)

	{
		std::vector<uint8_t> output;


		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length   
		output.push_back(0x05);
		//Command get state
		output.push_back(0x18);


		unsigned int temp_int1, temp_int2;




		temp_int1 = (unsigned int)speed;
		temp_int2 = (unsigned int)power;

		output.push_back(temp_int1 & 0xff);
		output.push_back((temp_int1 >> 8) & 0xff);
		output.push_back(temp_int2 & 0xff);
		output.push_back((temp_int2 >> 8) & 0xff);

		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;


		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];


		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);


		//Send message to the module
		port->write(output);


		ros::Duration(0.015).sleep();


		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		/*std::vector<uint8_t> input;


		while (input.empty()) {
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
                
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;*/
                return true;
	}

	//读取开口限位
	void gripper_serial::getOpenlimit(serial::Serial *port)
	{
		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length    
		output.push_back(1);
		//Command get state         
		output.push_back(0x13);
		//Checksum calculation
		unsigned int check_num = 0;
		int len = output[3] + 5;
		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];
		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);
		//Send message to the module 
		port->write(output);


		ros::Duration(0.015).sleep();


		std::string s1;
		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		//Read response
		std::vector<uint8_t> input;
		while (input.empty())
		{
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);


		int temp[10] = { 0 };
		for (int j = 0; j < 2; j++)
			temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

		//ROS_INFO("Gripper: No error detected");
		ROS_INFO_STREAM("gripper: openmax: " << temp[0] << " " << "gripper: openmin: " << temp[1]);
		openmax = float(temp[0]);
		openmin = float(temp[1]);
		//return(act_position_);
	}

	//读取当前开口
	void gripper_serial::getCopen(serial::Serial *port)

	{
		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length    
		output.push_back(1);
		//Command get state         
		output.push_back(0xD9);
		//Checksum calculation
		unsigned int check_num = 0;
		int len = output[3] + 5;
		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];
		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);
		//Send message to the module 
		port->write(output);


		ros::Duration(0.015).sleep();


		std::string s1;
		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		//Read response
		std::vector<uint8_t> input;
		while (input.empty())
		{
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);


		int temp[10] = { 0 };

		temp[0] = ((input[6] << 8) & 0xff00) + input[5];
		//ROS_INFO("Gripper: No error detected");

		ROS_INFO_STREAM("gripper: current open: " << temp[0]);
		curopen = float(temp[0]);
		//return(act_position_);
	}

	//读取当前状态
	uint8_t gripper_serial::getState(serial::Serial *port)

	{

		ROS_INFO("Reading current module state...");

		std::vector<uint8_t> output;

		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);

		//module id
		output.push_back(gripper_id_);

		//Data Length 
		output.push_back(1);

		//Command get state          
		output.push_back(0x41);

		//Checksum calculation
		unsigned int check_num = 0;
		int  len = output[3] + 5;
		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];
		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);
		//Send message to the module 

		port->write(output);

		ros::Duration(0.015).sleep();

		std::string s1;
		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		//Read response
		std::vector<uint8_t> input;
		const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(0.5);
        while (ros::ok() && input.size() < 13 && ros::WallTime::now() < deadline)
        {
            std::vector<uint8_t> part;
            port->read(part, (size_t)64);
            input.insert(input.end(), part.begin(), part.end());
        }
        if (input.size() < 13)
            return 0xff;


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);

		int temp;
		temp = input[7];
		int curopen;
		curopen = ((input[9] << 8) & 0xff00) + input[8];
		int power;
		power = ((input[11] << 8) & 0xff00) + input[10];

		int error[5] = { 0 };
		error[0] = input[6] & 0x01;
		error[1] = input[6] & 0x02;
		error[2] = input[6] & 0x04;
		error[3] = input[6] & 0x08;
		error[4] = input[6] & 0x10;

		if (error[0] == 1)
			ROS_INFO_STREAM("runing stop fault");

		if (error[1] == 2)
			ROS_INFO_STREAM("overheat fault");

		if (error[2] == 4)
			ROS_INFO_STREAM("Over Current Fault");

		if (error[3] == 8)
			ROS_INFO_STREAM("running fault");

		if (error[4] == 16)
			ROS_INFO_STREAM("communication fault");

		int state;
		state = input[5];
        motion_state_ = state;


		if (state == 1)
			ROS_INFO_STREAM("max in place");
		if (state == 2)
			ROS_INFO_STREAM("min in place");
		if (state == 3)
			ROS_INFO_STREAM("stop in place ");
		if (state == 4)
			ROS_INFO_STREAM("closing");
		if (state == 5)
			ROS_INFO_STREAM("openning");
		if (state == 6)
			ROS_INFO_STREAM("force control in place to stop");


		if ((unsigned int)input[6] == 0)
		{
			ROS_INFO_STREAM("Gripper: Temperature: " << temp << "[C] Current open: " << curopen << " Power" << power << "[g]");

			return((uint8_t)0x00);
		}
		else
		{
			return((uint8_t)0xff);
		}
	}

	//急停
	bool gripper_serial::setEstop(serial::Serial *port)
	{
		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length
		output.push_back(1);
		//Command get state
		output.push_back(0x016);
		//Checksum calculation
		unsigned int check_num = 0;
		int len = output[3] + 5;
		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];
		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);
		//Send message to the module
		port->write(output);
		ros::Duration(0.5 * 2).sleep();
		std::string s1;
		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);
		std::vector<uint8_t> input;
		const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(0.5);
        while (ros::ok() && input.size() < 7 && ros::WallTime::now() < deadline)
        {
            std::vector<uint8_t> part;
            port->read(part, (size_t)64);
            input.insert(input.end(), part.begin(), part.end());
        }
        if (input.size() < 7)
            return false;
		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//参数固化
	bool gripper_serial::setParam(serial::Serial *port)
	{
		std::vector<uint8_t> output;


		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length
		output.push_back(1);
		//Command get state
		output.push_back(0x01);

		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;


		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];


		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);


		//Send message to the module
		port->write(output);


		ros::Duration(0.5 * 2).sleep();


		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		std::vector<uint8_t> input;


		while (input.empty()) {
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}

	//清除故障
	bool gripper_serial::setFrsvd(serial::Serial *port)
	{
		std::vector<uint8_t> output;
		//message from master to module
		output.push_back(0xEB);
		output.push_back(0x90);
		//module id
		output.push_back(gripper_id_);
		//Data Length
		output.push_back(1);
		//Command get state
		output.push_back(0x17);

		//Checksum calculation

		unsigned int check_num = 0;
		int len = output[3] + 5;


		for (int i = 2; i < len - 1; i++)
			check_num = check_num + output[i];


		//Add checksum to the output buffer
		output.push_back(check_num & 0xff);


		//Send message to the module
		port->write(output);


		ros::Duration(0.5 * 2).sleep();


		std::string s1;

		for (int i = 0; i < output.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", output[i]);
			s1 = s1 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Write: " << s1);


		std::vector<uint8_t> input;


		while (input.empty()) {
			port->read(input, (size_t)64);
		}


		std::string s2;
		for (int i = 0; i < input.size(); ++i)
		{
			char str[16];
			sprintf(str, "%02X", input[i]);
			s2 = s2 + str + " ";
		}
		if (test_flags == 1)
			ROS_INFO_STREAM("Read: " << s2);
		int temp[10] = { 0 };
		for (int j = 0; j < 1; j++)
			temp[j] = input[5];
		if (temp[0] == 1)
			return true;
		else
			return false;
	}


	/////////////////////////////////////////////////////////////
	//CALLBACKS
	/////////////////////////////////////////////////////////////

	//设置开口限位
	bool gripper_serial::setOpenLimitCallback(inspire_gripper::set_openlimit::Request &req, inspire_gripper::set_openlimit::Response &res)
	{
		ROS_INFO("gripper: set open limit");
		if (req.openmax >= 0 && req.openmax <= 1000)
		{
			;
		}
		else
		{
			ROS_WARN("Gripper: openmax error!");
			res.openlimit_accepted = false;
			return false;
		}
		if (req.openmin >= 0 && req.openmin <= 1000)
		{
			;
		}
		else
		{
			ROS_WARN("Gripper: openmin error!");
			res.openlimit_accepted = false;
			return false;
		}

		if (req.openmax > req.openmin)
		{
			res.openlimit_accepted = setOpenLimit(com_port_, req.openmax, req.openmin);

		}
		else
		{
			ROS_WARN("Gripper: openmin >=openmax error!");
			res.openlimit_accepted = false;
		}
		return true;
	}

	//设置ID
	bool gripper_serial::setIDCallback(inspire_gripper::set_id::Request &req, inspire_gripper::set_id::Response &res)
	{
		ROS_INFO("gripper: reset id");
		if (req.id > 0 && req.id < 255)
		{
			res.idgrab = setID(com_port_, req.id);
		}
		else
		{
			ROS_INFO("Gripper: error id([1 254])!");
			res.idgrab = false;
		}
		return true;
	}

	//运动目标
	bool gripper_serial::moveTgtCallback(inspire_gripper::move_tgt::Request &req, inspire_gripper::move_tgt::Response &res)
	{

		if (req.movetgt >= 0 && req.movetgt <= 1000)
		{
			res.movetarget_accepted = moveTgt(com_port_, req.movetgt);
		}
		else
		{
			ROS_WARN("Gripper: movetarget error!");
			res.movetarget_accepted = false;
		}
		return true;
	}

	//运动松开
	bool gripper_serial::moveMaxCallback(inspire_gripper::move_max::Request &req, inspire_gripper::move_max::Response &res)
	{

		if (req.speed >= 1 && req.speed <= 1000)
		{
			res.movemax_accepted = moveMax(com_port_, req.speed);
		}
		else
		{
			ROS_WARN("Gripper: speed error!");
			res.movemax_accepted = false;
		}
		return true;
	}

	//运动抓取
	bool gripper_serial::moveMinCallback(inspire_gripper::move_min::Request &req, inspire_gripper::move_min::Response &res)
	{
		if (req.power >= 50 && req.power <= 1000)
		{
			;
		}
		else
		{
			ROS_WARN("Gripper: power error!");
			res.movemin_accepted = false;
			return false;
		}

		if (req.speed >= 1 && req.speed <= 1000)
		{
			res.movemin_accepted = moveMin(com_port_, req.speed, req.power);
		}
		else
		{
			ROS_WARN("Gripper: speed error!");
			res.movemin_accepted = false;
		}
		return true;
	}

	//运动持续抓取
	bool gripper_serial::moveMinHoldCallback(inspire_gripper::move_minhold::Request &req, inspire_gripper::move_minhold::Response &res)
	{
		if (req.power >= 50 && req.power <= 1000)
		{
			;
		}
		else
		{
			ROS_WARN("Gripper: power error!");
			res.moveminhold_accepted = false;
			//return false;
		}

		if (req.speed >= 1 && req.speed <= 1000)
		{
			res.moveminhold_accepted = moveMinHold(com_port_, req.speed, req.power);
		}
		else
		{
			ROS_WARN("Gripper: speed error!");
			res.moveminhold_accepted = false;
		}
		return true;
	}

	//读取开口限位
	bool gripper_serial::getOpenlimitCallback(inspire_gripper::get_openlimit::Request &req, inspire_gripper::get_openlimit::Response &res)
	{
		ROS_INFO("Gripper: Get openlimit request recieved");
		getOpenlimit(com_port_);
		res.openmax = openmax;
		res.openmin = openmin;
		return true;
	}

	//读取当前开口
	bool gripper_serial::getCopenCallback(inspire_gripper::get_copen::Request &req, inspire_gripper::get_copen::Response &res)
	{
		ROS_INFO
		("Gripper: Get Currentopen request recieved");

		getCopen(com_port_);

		res.curopen = curopen;
		return true;
	}

	//读取当前状态
	bool gripper_serial::getStateCallback(inspire_gripper::get_state::Request &req, inspire_gripper::get_state::Response &res)
	{
		ROS_INFO
		("Gripper: Get state request recieved");
		res.error_code = getState(com_port_);
        res.motion_state = res.error_code == 0 ? motion_state_ : -1;
		return true;
	}

	//急停
	bool gripper_serial::setEstopCallback(inspire_gripper::set_es::Request &req, inspire_gripper::set_es::Response &res)
	{
		ROS_INFO("Gripper: emergency stop Cmd recieved ");
		res.setes_accepted = setEstop(com_port_);
		//res.setparam_accepted = true;  
		return true;
	}

	//参数固化
	bool gripper_serial::setParamCallback(inspire_gripper::set_param::Request &req, inspire_gripper::set_param::Response &res)
	{
		ROS_INFO("Gripper: SetParam Cmd recieved ");
		res.setparam_accepted = setParam(com_port_);
		//res.setparam_accepted = true;  
		return true;
	}

	//清除故障
	bool gripper_serial::setFrsvdCallback(inspire_gripper::set_frsvd::Request &req, inspire_gripper::set_frsvd::Response &res)
	{
		ROS_INFO("Gripper: SetFaultResolved Cmd recieved ");
		res.setfrsvd_accepted = setFrsvd(com_port_);
		//res.setparam_accepted = true;  
		return true;
	}




	////////////////////////////////////////////////////
	//ADDITIONAL FUNCTIONS
	////////////////////////////////////////////////////

	float gripper_serial::IEEE_754_to_float(uint8_t *raw)
	{
		int sign = (raw[0] >> 7) ? -1 : 1;
		int8_t exponent = (raw[0] << 1) + (raw[1] >> 7) - 126;

		uint32_t fraction_bits = ((raw[1] & 0x7F) << 16) + (raw[2] << 8) + raw[3];

		float fraction = 0.5f;
		for (uint8_t ii = 0; ii < 24; ++ii)
			fraction += ldexpf((fraction_bits >> (23 - ii)) & 1, -(ii + 1));

		float significand = sign * fraction;

		return ldexpf(significand, exponent);
	}

	void gripper_serial::float_to_IEEE_754(float position, unsigned int *output_array)
	{
		unsigned char *p_byte = (unsigned char*)(&position);

		for (size_t i = 0; i < sizeof(float); i++)
			output_array[i] = (static_cast<unsigned int>(p_byte[i]));
	}

	uint16_t gripper_serial::CRC16(uint16_t crc, uint16_t data)
	{
		const uint16_t tbl[256] = {
		0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
		0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
		0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
		0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
		0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
		0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
		0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
		0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
		0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
		0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
		0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
		0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
		0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
		0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
		0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
		0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
		0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
		0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
		0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
		0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
		0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
		0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
		0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
		0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
		0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
		0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
		0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
		0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
		0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
		0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
		0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
		0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
		};

		return(((crc & 0xFF00) >> 8) ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)]);
	}
}

#endif
