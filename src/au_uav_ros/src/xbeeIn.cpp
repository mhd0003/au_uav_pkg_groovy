#include "au_uav_ros/xbeeIn.h"
using namespace au_uav_ros;

void XbeeIn::run(void) { //TODO POSSIBLY use timers and custom callback queue to exit normally
	while (fd == -1 && ros::ok()) {
		ros::spinOnce();	
	}
	if (fd != -1) {
		int* fd_ptr = &fd;
		serial_wait((void *)fd_ptr);
	} else {
		return;
	}
}

void XbeeIn::init(ros::NodeHandle _n) {
	n = _n;
	setup();
}

void XbeeIn::setup() {
	shutdownTopic = n.subscribe("component_shutdown", 1000, &XbeeIn::component_shutdown, this);

	initCommsService = n.advertiseService("init_incomms", &XbeeIn::init_comms, this);
	closeCommsService = n.advertiseService("close_incomms", &XbeeIn::close_comms, this);

	telemetryTopic = n.advertise<au_uav_ros::Telemetry>("telemetry", 1000);

	sendAckClient = n.serviceClient<au_uav_ros::Ack>("ack_recieved");
	recieveAckService = n.advertiseService("add_ack", &XbeeIn::add_ack, this);

	fd = -1;
	baud = 57600;
	sysid = -1;
	compid = 110;
	serial_compid = 0;
	port = "/dev/ttyUSB0";
	pc2serial = true;
	updateIndex = 0;
	WPSendSeqNum = 0;
}

void XbeeIn::component_shutdown(const std_msgs::String::ConstPtr &msg) {
	close_port(fd);
	ros::shutdown();
}

bool XbeeIn::init_comms(InitComms::Request &req, InitComms::Response &res) {
	fd = open_port(port);
	if (fd == -1) {
		res.error = "Port did not open";
		return false;
	}
	bool setup = setup_port(fd, baud, 8, 1, false, false);
	if (!setup) {
		res.error = "Failed to setup port";
		return false;
	}
	res.error = "None";
	return true;
	
} 


bool XbeeIn::close_comms(CloseComms::Request &req, CloseComms::Response &res) {
	close_port(fd);
	res.error = "None";
	return true; //TODO Check return of close_port
}


bool XbeeIn::add_ack(Ack::Request &req, Ack::Response &res) {
	return true; //TODO keep track of ack requests
}

int XbeeIn::open_port(std::string& port)
{
	int _fd; /* File descriptor for the port */
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (_fd == -1)
	{
		/* Could not open the port. */
		ROS_ERROR("IN: COULD NOT OPEN PORT");
		return(-1);
	}
	else
	{
		fcntl(_fd, F_SETFL, 0);
	}
	
	return (_fd);
}

bool XbeeIn::setup_port(int _fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;
	
	struct termios  config;
	if(!isatty(_fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %s is NOT a serial port\n", port.c_str());
		return false;
	}
	if(tcgetattr(_fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of port %s\n", port.c_str());
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;
	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 0; //TODO changed from 1 to 0 it was blocking read so it could never get shutdown if no message was sent
	config.c_cc[VTIME] = 10; // was 0
	
	// Get the current options for the port
	//tcgetattr(fd, &options);
	
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, falling back to 115200 8N1 default rate.\n", baud);
			cfsetispeed(&config, B115200);
			cfsetospeed(&config, B115200);
			
			break;
	}
	
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(_fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of port %s\n", port.c_str());
		return false;
	}
	return true;
}

bool XbeeIn::close_port(int _fd)
{
	close(_fd);
	return true;
}

bool XbeeIn::convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate) {
	//tUpdate.planeID = mavlink_ros.planeID;
	
	tUpdate.currentLatitude = (mavMessage.au_lat) / 10000000.0;	  /// Lattitude * 10**7 so have to divide by 10^7
	tUpdate.currentLongitude = (mavMessage.au_lng) / 10000000.0;	  /// Longitude * 10**7 so have to divide by 10^7
	tUpdate.currentAltitude = (mavMessage.au_alt) / 100.0; 	  /// Altitude in cm so divide by 100 to get meters	
	
	tUpdate.destLatitude = (mavMessage.au_target_lat) / 10000000.0;  /// Lattitude * 10**7 so have to divide by 10^7
	tUpdate.destLongitude = (mavMessage.au_target_lng) / 10000000.0; /// Longitude * 10**7 so have to divide by 10^7
	tUpdate.destAltitude = (mavMessage.au_target_alt) / 100.0; 	  /// Altitude in cm so divide by 100 to get meters

	tUpdate.groundSpeed = (mavMessage.au_ground_speed) / 100.0; /// Originally in cm / sec so have to convert to mph

	tUpdate.distanceToDestination = mavMessage.au_distance; 	  /// Distance between plane and next waypoint in meters.

	tUpdate.targetBearing = mavMessage.au_target_bearing / 100;	  /// This is the direction to the next waypoint or loiter center in degrees

	tUpdate.currentWaypointIndex = mavMessage.au_target_wp_index;   /// The current waypoint index

	tUpdate.telemetryHeader.seq = ++updateIndex;
	tUpdate.telemetryHeader.stamp = ros::Time::now();
	return true;
}

void* XbeeIn::serial_wait(void* serial_ptr)
{
	int _fd = *((int*) serial_ptr);	

	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	
	// Blocking wait for new data
	while (ros::ok())
	{
		ros::spinOnce();
		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;
		//tcflush(fd, TCIFLUSH);
		serialPort.lock();
		int num = read(_fd, &cp, 1);
		serialPort.unlock();
		if (num > 0)
		{
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
			{
				/**if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				if (debug)
				{
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				} */
			}
			lastStatus = status;
		}
		else
		{
			//if (!silent) fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
		}
		
		// If a message could be decoded, handle it
		if(msgReceived)
		{		
			sysid = message.sysid;
			serial_compid = message.compid;

			if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
			{
				//ROS_INFO("Delay: %d", delay);
				//mavlink_heartbeat_t receivedHeartbeat;
				//mavlink_msg_heartbeat_decode(&message, &receivedHeartbeat);
				//ROS_INFO("Received heartbeat with ID #%d (type:%d | AP:%d | base:%d | custom:%d | status:%d)\n", message.msgid, receivedHeartbeat.type, receivedHeartbeat.autopilot, receivedHeartbeat.base_mode, receivedHeartbeat.custom_mode, receivedHeartbeat.system_status);
			}
			if(message.msgid == MAVLINK_MSG_ID_AU_UAV)
			{
				//ROS_INFO("Received AU_UAV message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
				
				au_uav_ros::Telemetry tUpdate;
				mavlink_au_uav_t myMSG;
				mavlink_msg_au_uav_decode(&message, &myMSG);		
				convertMavlinkTelemetryToROS(myMSG, tUpdate);			
				tUpdate.planeID = message.sysid;
				ROS_INFO("Received telemetry message from UAV[#%d] (lat:%f|lng:%f|alt:%f)", tUpdate.planeID, tUpdate.currentLatitude, tUpdate.currentLongitude, tUpdate.currentAltitude);
				telemetryTopic.publish(tUpdate);	// publish ROS telemetry message to ROS topic /telemetry
			}
			//TODO: add code here to test if received ack corresponsed to desired target_wp, if not retransmit target_wp
			
			if (message.msgid == MAVLINK_MSG_ID_MISSION_ACK) 
			{
				mavlink_mission_ack_t mission_ack;
				mavlink_msg_mission_ack_decode(&message, &mission_ack);
				//Ack a;
				//a.request.planeID = message.sysid;
				//sendAckClient.call(a);
			}
			if (message.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
				mavlink_param_value_t value_ack;
				mavlink_msg_param_value_decode(&message, &value_ack);
			}
			
		}
	}
	return NULL;
} //end serial_wait
int main(int argc, char **argv) {
	ros::init(argc, argv, "XbeeIn");
	ros::NodeHandle n;
	XbeeIn x;
	x.init(n);
	x.run();
	return 0;
}
