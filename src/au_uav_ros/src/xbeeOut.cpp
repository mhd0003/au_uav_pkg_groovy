#include "au_uav_ros/xbeeOut.h"
using namespace au_uav_ros;
void XbeeOut::run(void) {
	ros::spin();
}

void XbeeOut::init(ros::NodeHandle _n) {
	n = _n;
	setup();
}

void XbeeOut::setup() {
	shutdownTopic = n.subscribe("component_shutdown", 1000, &XbeeOut::component_shutdown, this);
	initCommsService = n.advertiseService("init_outcomms", &XbeeOut::init_comms, this);
	closeCommsService = n.advertiseService("close_outcomms", &XbeeOut::close_comms, this);

	commandTopic = n.subscribe("commands", 1000, &XbeeOut::commands, this);

	sendAckClient = n.serviceClient<au_uav_ros::Ack>("add_ack");
	recieveAckService = n.advertiseService("ack_recieved", &XbeeOut::ack_recieved, this);

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

void XbeeOut::component_shutdown(const std_msgs::String::ConstPtr &msg) {
		close_port(fd);
		ros::shutdown();
}

bool XbeeOut::init_comms(InitComms::Request &req, InitComms::Response &res) {
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

bool XbeeOut::close_comms(CloseComms::Request &req, CloseComms::Response &res) {
	close_port(fd);
	res.error = "None";
	return true; //TODO Check return of close_port
}

void XbeeOut::commands(const au_uav_ros::Command &cmd) {
	if (!cmd.sim) {
		sysid = cmd.planeID;
		pendingAcks.push_back(cmd);
		Ack a;
		a.request.planeID = cmd.planeID;
		sendAckClient.call(a);
		WPSendSeqNum++;
		mavlink_message_t mavlinkMsg;
		// Refer to message_definitions for parameter explination
		switch (cmd.commandID) {
			case COMMAND_NORMAL_WP: //waypoint command
			case COMMAND_AVOID_WP: //waypoint command
				mavlink_msg_mission_item_pack(sysid, compid, &mavlinkMsg, 
							sysid, serial_compid, WPSendSeqNum, 
							MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 
							2, 0, 20.0, 100.0, 1.0, 0.0, 
							cmd.latitude, cmd.longitude, cmd.altitude);
				break;
			case COMMAND_SET_ID: //change id commmand
				mavlink_msg_param_set_pack(serial_compid, compid, &mavlinkMsg, sysid, compid, 
							 "SYSID_THISMAV", (unsigned)cmd.param, MAVLINK_TYPE_UINT64_T);
				break;
			default:
				break;
		} 
		// Send message over serial port TODO Get ACK
		static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
		int messageLength = mavlink_msg_to_send_buffer(buffer, &mavlinkMsg);
		//if (debug) printf("Writing %d bytes\n", messageLength);
		serialPort.lock();
		int written = write(fd, (char*)buffer, messageLength);
		serialPort.unlock();
		tcflush(fd, TCOFLUSH);
		if (messageLength != written) fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength);

	}
	else {
	// command is for simulated UAV
	}
}

int XbeeOut::open_port(std::string& port)
{
	int _fd; /* File descriptor for the port */
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (_fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		fcntl(_fd, F_SETFL, 0);
	}
	
	return (_fd);
}

bool XbeeOut::setup_port(int _fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
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

bool XbeeOut::close_port(int _fd)
{
	close(_fd);
	return true;
}

bool XbeeOut::convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate) {
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

bool XbeeOut::ack_recieved(Ack::Request &req, Ack::Response &res) {
	bool found = false;
	std::list<Command>::iterator i;
	for (i = pendingAcks.begin(); i != pendingAcks.end(); i++) {
		if (i->planeID == req.planeID) { //TODO use command seq to verify command
			found = true;
			break;
		}
	}
	if (found) {
		ROS_ERROR("ACK RECIEVED");
		//pendingAcks.remove(i);
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "XbeeOut");
	ros::NodeHandle n;
	XbeeOut x;
	x.init(n);
	x.run();
	return 0;
}

