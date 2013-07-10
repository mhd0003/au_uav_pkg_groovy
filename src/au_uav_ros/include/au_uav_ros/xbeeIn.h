#ifndef _XBEE_IN_H_
#define _XBEE_IN_H_

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string>
#include <inttypes.h>
#include <fstream>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#ifdef __linux
#include <sys/ioctl.h>
#endif

#include <glib.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

//Standard C++ headers
#include <sstream>

//ROS headers
#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "au_uav_ros/standardDefs.h"

#include "au_uav_ros/Ack.h"
#include "au_uav_ros/Command.h"
#include "au_uav_ros/CloseComms.h"
#include "au_uav_ros/InitComms.h"
#include "au_uav_ros/Telemetry.h"

#include "mavlink/v1.0/common/mavlink.h"

namespace au_uav_ros{
	class XbeeIn {
	private:
		int baud;
		int sysid;
		int compid;
		int serial_compid;
		std::string port;
		bool pc2serial;
		int fd;
		int updateIndex;
		int WPSendSeqNum;
		int myMessage[256];

		//Services and topics from master
		ros::Subscriber shutdownTopic;

		//Services and topics from GuiIntefacer
		ros::ServiceServer initCommsService;
		ros::ServiceServer closeCommsService;

		//Broadcasted topics
		ros::Publisher telemetryTopic;

		//Services and Topics from XbeeOut
		ros::ServiceServer recieveAckService;
		ros::ServiceClient sendAckClient;

		//Xbee internals
		ros::NodeHandle n;
	public:
		void run(void);
		void init(ros::NodeHandle _n);
		void setup();
		void component_shutdown(const std_msgs::String::ConstPtr &msg);
		bool init_comms(InitComms::Request &req, InitComms::Response &res);
		bool close_comms(CloseComms::Request &req, CloseComms::Response &res);
		bool add_ack(Ack::Request &req, Ack::Response &res);
		bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
		int open_port(std::string& port);
		bool close_port(int fd);
		bool convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate);
		void* serial_wait(void* serial_ptr);
	};
}
#endif
