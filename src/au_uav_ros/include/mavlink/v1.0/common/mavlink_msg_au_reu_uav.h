// MESSAGE AU_REU_UAV PACKING

#define MAVLINK_MSG_ID_AU_REU_UAV 199

typedef struct __mavlink_au_reu_uav_t
{
 int32_t au_lat; ///< Latitude, expressed as * 1E7
 int32_t au_lon; ///< Longitude, expressed as * 1E7
 int32_t au_alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 int32_t au_target_lat; ///< Target waypoint latitude expressed as 1E7
 int32_t au_target_lon; ///< Target waypoint longitude expressed     as 1E7
 int32_t au_target_alt; ///< Target waypoint altitude expressed as 1000 (millimeters)
 int32_t au_ground_speed; ///< Ground speed in cm/sec
 int32_t au_airspeed; ///< Ground speed in cm/sec
 int32_t au_target_bearing; ///< Target bearing torward next waypoint in degress (0 to 360)
 int32_t au_distance; ///< Distance to next waypoint expressed in meters
 uint8_t au_target_wp_index; ///< The waypoint index of the target wp
} mavlink_au_reu_uav_t;

#define MAVLINK_MSG_ID_AU_REU_UAV_LEN 41
#define MAVLINK_MSG_ID_199_LEN 41



#define MAVLINK_MESSAGE_INFO_AU_REU_UAV { \
	"AU_REU_UAV", \
	11, \
	{  { "au_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_au_reu_uav_t, au_lat) }, \
         { "au_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_au_reu_uav_t, au_lon) }, \
         { "au_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_au_reu_uav_t, au_alt) }, \
         { "au_target_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_au_reu_uav_t, au_target_lat) }, \
         { "au_target_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_au_reu_uav_t, au_target_lon) }, \
         { "au_target_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_au_reu_uav_t, au_target_alt) }, \
         { "au_ground_speed", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_au_reu_uav_t, au_ground_speed) }, \
         { "au_airspeed", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_au_reu_uav_t, au_airspeed) }, \
         { "au_target_bearing", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_au_reu_uav_t, au_target_bearing) }, \
         { "au_distance", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_au_reu_uav_t, au_distance) }, \
         { "au_target_wp_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_au_reu_uav_t, au_target_wp_index) }, \
         } \
}


/**
 * @brief Pack a au_reu_uav message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param au_lat Latitude, expressed as * 1E7
 * @param au_lon Longitude, expressed as * 1E7
 * @param au_alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param au_target_lat Target waypoint latitude expressed as 1E7
 * @param au_target_lon Target waypoint longitude expressed     as 1E7
 * @param au_target_alt Target waypoint altitude expressed as 1000 (millimeters)
 * @param au_ground_speed Ground speed in cm/sec
 * @param au_airspeed Ground speed in cm/sec
 * @param au_target_bearing Target bearing torward next waypoint in degress (0 to 360)
 * @param au_distance Distance to next waypoint expressed in meters
 * @param au_target_wp_index The waypoint index of the target wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_au_reu_uav_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t au_lat, int32_t au_lon, int32_t au_alt, int32_t au_target_lat, int32_t au_target_lon, int32_t au_target_alt, int32_t au_ground_speed, int32_t au_airspeed, int32_t au_target_bearing, int32_t au_distance, uint8_t au_target_wp_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[41];
	_mav_put_int32_t(buf, 0, au_lat);
	_mav_put_int32_t(buf, 4, au_lon);
	_mav_put_int32_t(buf, 8, au_alt);
	_mav_put_int32_t(buf, 12, au_target_lat);
	_mav_put_int32_t(buf, 16, au_target_lon);
	_mav_put_int32_t(buf, 20, au_target_alt);
	_mav_put_int32_t(buf, 24, au_ground_speed);
	_mav_put_int32_t(buf, 28, au_airspeed);
	_mav_put_int32_t(buf, 32, au_target_bearing);
	_mav_put_int32_t(buf, 36, au_distance);
	_mav_put_uint8_t(buf, 40, au_target_wp_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 41);
#else
	mavlink_au_reu_uav_t packet;
	packet.au_lat = au_lat;
	packet.au_lon = au_lon;
	packet.au_alt = au_alt;
	packet.au_target_lat = au_target_lat;
	packet.au_target_lon = au_target_lon;
	packet.au_target_alt = au_target_alt;
	packet.au_ground_speed = au_ground_speed;
	packet.au_airspeed = au_airspeed;
	packet.au_target_bearing = au_target_bearing;
	packet.au_distance = au_distance;
	packet.au_target_wp_index = au_target_wp_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 41);
#endif

	msg->msgid = MAVLINK_MSG_ID_AU_REU_UAV;
	return mavlink_finalize_message(msg, system_id, component_id, 41, 46);
}

/**
 * @brief Pack a au_reu_uav message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param au_lat Latitude, expressed as * 1E7
 * @param au_lon Longitude, expressed as * 1E7
 * @param au_alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param au_target_lat Target waypoint latitude expressed as 1E7
 * @param au_target_lon Target waypoint longitude expressed     as 1E7
 * @param au_target_alt Target waypoint altitude expressed as 1000 (millimeters)
 * @param au_ground_speed Ground speed in cm/sec
 * @param au_airspeed Ground speed in cm/sec
 * @param au_target_bearing Target bearing torward next waypoint in degress (0 to 360)
 * @param au_distance Distance to next waypoint expressed in meters
 * @param au_target_wp_index The waypoint index of the target wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_au_reu_uav_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t au_lat,int32_t au_lon,int32_t au_alt,int32_t au_target_lat,int32_t au_target_lon,int32_t au_target_alt,int32_t au_ground_speed,int32_t au_airspeed,int32_t au_target_bearing,int32_t au_distance,uint8_t au_target_wp_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[41];
	_mav_put_int32_t(buf, 0, au_lat);
	_mav_put_int32_t(buf, 4, au_lon);
	_mav_put_int32_t(buf, 8, au_alt);
	_mav_put_int32_t(buf, 12, au_target_lat);
	_mav_put_int32_t(buf, 16, au_target_lon);
	_mav_put_int32_t(buf, 20, au_target_alt);
	_mav_put_int32_t(buf, 24, au_ground_speed);
	_mav_put_int32_t(buf, 28, au_airspeed);
	_mav_put_int32_t(buf, 32, au_target_bearing);
	_mav_put_int32_t(buf, 36, au_distance);
	_mav_put_uint8_t(buf, 40, au_target_wp_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 41);
#else
	mavlink_au_reu_uav_t packet;
	packet.au_lat = au_lat;
	packet.au_lon = au_lon;
	packet.au_alt = au_alt;
	packet.au_target_lat = au_target_lat;
	packet.au_target_lon = au_target_lon;
	packet.au_target_alt = au_target_alt;
	packet.au_ground_speed = au_ground_speed;
	packet.au_airspeed = au_airspeed;
	packet.au_target_bearing = au_target_bearing;
	packet.au_distance = au_distance;
	packet.au_target_wp_index = au_target_wp_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 41);
#endif

	msg->msgid = MAVLINK_MSG_ID_AU_REU_UAV;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 41, 46);
}

/**
 * @brief Encode a au_reu_uav struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param au_reu_uav C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_au_reu_uav_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_au_reu_uav_t* au_reu_uav)
{
	return mavlink_msg_au_reu_uav_pack(system_id, component_id, msg, au_reu_uav->au_lat, au_reu_uav->au_lon, au_reu_uav->au_alt, au_reu_uav->au_target_lat, au_reu_uav->au_target_lon, au_reu_uav->au_target_alt, au_reu_uav->au_ground_speed, au_reu_uav->au_airspeed, au_reu_uav->au_target_bearing, au_reu_uav->au_distance, au_reu_uav->au_target_wp_index);
}

/**
 * @brief Send a au_reu_uav message
 * @param chan MAVLink channel to send the message
 *
 * @param au_lat Latitude, expressed as * 1E7
 * @param au_lon Longitude, expressed as * 1E7
 * @param au_alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param au_target_lat Target waypoint latitude expressed as 1E7
 * @param au_target_lon Target waypoint longitude expressed     as 1E7
 * @param au_target_alt Target waypoint altitude expressed as 1000 (millimeters)
 * @param au_ground_speed Ground speed in cm/sec
 * @param au_airspeed Ground speed in cm/sec
 * @param au_target_bearing Target bearing torward next waypoint in degress (0 to 360)
 * @param au_distance Distance to next waypoint expressed in meters
 * @param au_target_wp_index The waypoint index of the target wp
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_au_reu_uav_send(mavlink_channel_t chan, int32_t au_lat, int32_t au_lon, int32_t au_alt, int32_t au_target_lat, int32_t au_target_lon, int32_t au_target_alt, int32_t au_ground_speed, int32_t au_airspeed, int32_t au_target_bearing, int32_t au_distance, uint8_t au_target_wp_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[41];
	_mav_put_int32_t(buf, 0, au_lat);
	_mav_put_int32_t(buf, 4, au_lon);
	_mav_put_int32_t(buf, 8, au_alt);
	_mav_put_int32_t(buf, 12, au_target_lat);
	_mav_put_int32_t(buf, 16, au_target_lon);
	_mav_put_int32_t(buf, 20, au_target_alt);
	_mav_put_int32_t(buf, 24, au_ground_speed);
	_mav_put_int32_t(buf, 28, au_airspeed);
	_mav_put_int32_t(buf, 32, au_target_bearing);
	_mav_put_int32_t(buf, 36, au_distance);
	_mav_put_uint8_t(buf, 40, au_target_wp_index);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AU_REU_UAV, buf, 41, 46);
#else
	mavlink_au_reu_uav_t packet;
	packet.au_lat = au_lat;
	packet.au_lon = au_lon;
	packet.au_alt = au_alt;
	packet.au_target_lat = au_target_lat;
	packet.au_target_lon = au_target_lon;
	packet.au_target_alt = au_target_alt;
	packet.au_ground_speed = au_ground_speed;
	packet.au_airspeed = au_airspeed;
	packet.au_target_bearing = au_target_bearing;
	packet.au_distance = au_distance;
	packet.au_target_wp_index = au_target_wp_index;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AU_REU_UAV, (const char *)&packet, 41, 46);
#endif
}

#endif

// MESSAGE AU_REU_UAV UNPACKING


/**
 * @brief Get field au_lat from au_reu_uav message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field au_lon from au_reu_uav message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field au_alt from au_reu_uav message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field au_target_lat from au_reu_uav message
 *
 * @return Target waypoint latitude expressed as 1E7
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_target_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field au_target_lon from au_reu_uav message
 *
 * @return Target waypoint longitude expressed     as 1E7
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_target_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field au_target_alt from au_reu_uav message
 *
 * @return Target waypoint altitude expressed as 1000 (millimeters)
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_target_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field au_ground_speed from au_reu_uav message
 *
 * @return Ground speed in cm/sec
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_ground_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field au_airspeed from au_reu_uav message
 *
 * @return Ground speed in cm/sec
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field au_target_bearing from au_reu_uav message
 *
 * @return Target bearing torward next waypoint in degress (0 to 360)
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_target_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field au_distance from au_reu_uav message
 *
 * @return Distance to next waypoint expressed in meters
 */
static inline int32_t mavlink_msg_au_reu_uav_get_au_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field au_target_wp_index from au_reu_uav message
 *
 * @return The waypoint index of the target wp
 */
static inline uint8_t mavlink_msg_au_reu_uav_get_au_target_wp_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Decode a au_reu_uav message into a struct
 *
 * @param msg The message to decode
 * @param au_reu_uav C-struct to decode the message contents into
 */
static inline void mavlink_msg_au_reu_uav_decode(const mavlink_message_t* msg, mavlink_au_reu_uav_t* au_reu_uav)
{
#if MAVLINK_NEED_BYTE_SWAP
	au_reu_uav->au_lat = mavlink_msg_au_reu_uav_get_au_lat(msg);
	au_reu_uav->au_lon = mavlink_msg_au_reu_uav_get_au_lon(msg);
	au_reu_uav->au_alt = mavlink_msg_au_reu_uav_get_au_alt(msg);
	au_reu_uav->au_target_lat = mavlink_msg_au_reu_uav_get_au_target_lat(msg);
	au_reu_uav->au_target_lon = mavlink_msg_au_reu_uav_get_au_target_lon(msg);
	au_reu_uav->au_target_alt = mavlink_msg_au_reu_uav_get_au_target_alt(msg);
	au_reu_uav->au_ground_speed = mavlink_msg_au_reu_uav_get_au_ground_speed(msg);
	au_reu_uav->au_airspeed = mavlink_msg_au_reu_uav_get_au_airspeed(msg);
	au_reu_uav->au_target_bearing = mavlink_msg_au_reu_uav_get_au_target_bearing(msg);
	au_reu_uav->au_distance = mavlink_msg_au_reu_uav_get_au_distance(msg);
	au_reu_uav->au_target_wp_index = mavlink_msg_au_reu_uav_get_au_target_wp_index(msg);
#else
	memcpy(au_reu_uav, _MAV_PAYLOAD(msg), 41);
#endif
}
