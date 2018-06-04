#include <iostream>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "uart_driver.h"
#include <ls01d/LaserRanges.h>
#include <ls01d/CMD.h>

io_driver driver;
bool is_scan_stop;

bool stopMotor(ls01d::CMD::Request  &req,
         ls01d::CMD::Response &res)
{
	if (!is_scan_stop)
	{
		driver.StopScan(STOP_DATA);
		driver.StopScan(STOP_MOTOR);
		is_scan_stop = true;
		ROS_WARN("stop scan");
		res.data = "stop";
	}
	else
		res.data = "laser already stopped";

	return true;
}

bool startMotor(ls01d::CMD::Request  &req,
         ls01d::CMD::Response &res)
{
	if (is_scan_stop)
	{
		driver.StartScan();
		is_scan_stop = false;
		ROS_WARN("start scan");
		res.data = "start";
	}
	else
		res.data ="laser already started";
  
	return true;
}

int main(int argc, char *argv[])
{
    // ROS节点初始化
  ros::init(argc, argv, "talker");
  
  // 创建节点句柄
  ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
  // 获取节点相关参数
  std::string scan_topic, laser_link, port;
  int angle_disable_max, angle_disable_min;
  bool zero_as_max, min_as_zero;

  if (!private_nh_.getParam("scan_topic",scan_topic))
	scan_topic = "scan";

  if (!private_nh_.getParam("laser_link",laser_link))
	laser_link = "base_laser_link";
  
  if (!private_nh_.getParam("serial_port",port))
	port = "/dev/ttyUSB0";
  
  if (!private_nh_.getParam("angle_disable_min",angle_disable_min))
	angle_disable_min = -1;
  
  if (!private_nh_.getParam("angle_disable_max",angle_disable_max))
	angle_disable_max = -1;
  
  if (!private_nh_.getParam("zero_as_max",zero_as_max))
	zero_as_max = true;
  
  if (!private_nh_.getParam("min_as_zero",min_as_zero))
	min_as_zero = false;

	ROS_WARN("trucated angle range from %d to %d", angle_disable_min, angle_disable_max);

	ROS_WARN("zero as max: %d, min_as_zero: %d", zero_as_max, min_as_zero);
  

  // 配置雷达串口并打开 
  int ret;
  ret = driver.OpenSerial(port.c_str(), B230400);
  if (ret < 0)
  {
	ROS_FATAL("could not open port:%s", port.c_str());
	return 0;
  }
  else
  {
	ROS_INFO("open port:%s successfully", port.c_str());
  }

  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
	ros::ServiceServer stop_srv = n.advertiseService("stop_scan", stopMotor);
	ros::ServiceServer start_srv = n.advertiseService("start_scan", startMotor);

  // 设置循环的频率
  ros::Rate loop_rate(10);

  // 启动雷达扫描 
  driver.StartScan();
  ROS_INFO("Send start command successfully");

  // 用于解析雷达数据的变量 
  double angle[PACKLEN + 10];
  double distance[PACKLEN + 10];
  double data[PACKLEN + 10];
  double data_intensity[PACKLEN + 10];
  double speed;
  int count = 0;
  int point_num = 360;
  int half_point = point_num / 2;
	
  while (ros::ok())
  {
		if (is_scan_stop)
		{
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		
		sensor_msgs::LaserScan scan_msg;
		scan_msg.header.stamp = ros::Time::now();
		scan_msg.header.frame_id = laser_link;
		scan_msg.angle_min = 0.0;
		scan_msg.angle_max = 2 * M_PI;
		scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / point_num;
		scan_msg.range_min = 0.1;
		scan_msg.range_max = 10.0;

		memset(data, 0, sizeof(data));
		int ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
		scan_msg.intensities.resize(ret);
		scan_msg.ranges.resize(ret);

		double max_range = scan_msg.range_max - 0.2;
		// extract the first angle alone
		if ((distance[0]== 0) && (zero_as_max))
			scan_msg.ranges[0] = max_range;
		else if (distance[0] == 0){
			if (min_as_zero)
				scan_msg.ranges[0] = 0.0;
			else
				scan_msg.ranges[0] = std::numeric_limits<float>::infinity();
		}
		else
			scan_msg.ranges[0] = distance[0] / 1000.0;

		// right hand rule first part (1-180) 
		for (int i = 1; i <= (half_point-1); i++)
		{
			if ((i>=angle_disable_min) && (i<angle_disable_max))
			{
				if (min_as_zero)
					scan_msg.ranges[i] = 0.0;
				else
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
			}
			else if ((distance[point_num-1-i] == 0) && (zero_as_max))
				scan_msg.ranges[i] = scan_msg.range_max - 0.2;
			else if (distance[point_num-1-i] == 0)
			{
				if (min_as_zero)
					scan_msg.ranges[i] = 0.0;
				else
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
			}
			else
				scan_msg.ranges[i] = distance[point_num-1-i] / 1000.0;
			
			scan_msg.intensities[i] = floor(angle[point_num-1]);
		}

		// right hand rule second part (180-360)
		for (int i = half_point; i < point_num; i++)
		{
			if ((i>=angle_disable_min) && (i<angle_disable_max))
			{
				if (min_as_zero)
					scan_msg.ranges[i] = 0.0;
				else
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
			}
			else if ((distance[point_num-1-i] == 0) && (zero_as_max))
				scan_msg.ranges[i] = scan_msg.range_max - 0.2;
			else if (distance[point_num-1-i] == 0)
			{
				if (min_as_zero)
					scan_msg.ranges[i] = 0.0;
				else
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
			}
			else
				scan_msg.ranges[i] = distance[point_num-1-i] / 1000.0;
			
			scan_msg.intensities[i] = floor(angle[point_num-1-i]);
		}

		scan_pub.publish(scan_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	// close serial port when exit this rosnode   
	driver.StopScan(STOP_DATA);
	driver.StopScan(STOP_MOTOR);
	driver.CloseSerial();
	ROS_FATAL("Keyboard Interrupt, ls01d stop!");
	return 0;
}