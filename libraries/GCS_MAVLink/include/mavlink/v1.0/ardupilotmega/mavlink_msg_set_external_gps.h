// MESSAGE SET_EXTERNAL_GPS PACKING

#define MAVLINK_MSG_ID_SET_EXTERNAL_GPS 230

typedef struct __mavlink_set_external_gps_t
{
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as * 1E7*/
 int32_t lon; /*< Longitude, expressed as * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters)*/
 uint16_t hdop; /*< HDOP of GPS*/
 int16_t vx; /*< Ground X Speed (Latitude), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude), expressed as m/s * 100*/
 uint8_t gps_status; /*< GPS Status*/
 uint8_t num_sats; /*< Number of satellites*/
} mavlink_set_external_gps_t;

#define MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN 30
#define MAVLINK_MSG_ID_230_LEN 30

#define MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC 216
#define MAVLINK_MSG_ID_230_CRC 216



#define MAVLINK_MESSAGE_INFO_SET_EXTERNAL_GPS { \
	"SET_EXTERNAL_GPS", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_set_external_gps_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_external_gps_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_set_external_gps_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_set_external_gps_t, alt) }, \
         { "hdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_set_external_gps_t, hdop) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_set_external_gps_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_set_external_gps_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_set_external_gps_t, vz) }, \
         { "gps_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_set_external_gps_t, gps_status) }, \
         { "num_sats", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_set_external_gps_t, num_sats) }, \
         } \
}


/**
 * @brief Pack a set_external_gps message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gps_status GPS Status
 * @param num_sats Number of satellites
 * @param hdop HDOP of GPS
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_external_gps_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t gps_status, uint8_t num_sats, uint16_t hdop, uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, hdop);
	_mav_put_int16_t(buf, 22, vx);
	_mav_put_int16_t(buf, 24, vy);
	_mav_put_int16_t(buf, 26, vz);
	_mav_put_uint8_t(buf, 28, gps_status);
	_mav_put_uint8_t(buf, 29, num_sats);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#else
	mavlink_set_external_gps_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.hdop = hdop;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.gps_status = gps_status;
	packet.num_sats = num_sats;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_EXTERNAL_GPS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
}

/**
 * @brief Pack a set_external_gps message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_status GPS Status
 * @param num_sats Number of satellites
 * @param hdop HDOP of GPS
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_external_gps_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t gps_status,uint8_t num_sats,uint16_t hdop,uint64_t time_usec,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, hdop);
	_mav_put_int16_t(buf, 22, vx);
	_mav_put_int16_t(buf, 24, vy);
	_mav_put_int16_t(buf, 26, vz);
	_mav_put_uint8_t(buf, 28, gps_status);
	_mav_put_uint8_t(buf, 29, num_sats);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#else
	mavlink_set_external_gps_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.hdop = hdop;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.gps_status = gps_status;
	packet.num_sats = num_sats;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_EXTERNAL_GPS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
}

/**
 * @brief Encode a set_external_gps struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_external_gps C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_external_gps_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_external_gps_t* set_external_gps)
{
	return mavlink_msg_set_external_gps_pack(system_id, component_id, msg, set_external_gps->gps_status, set_external_gps->num_sats, set_external_gps->hdop, set_external_gps->time_usec, set_external_gps->lat, set_external_gps->lon, set_external_gps->alt, set_external_gps->vx, set_external_gps->vy, set_external_gps->vz);
}

/**
 * @brief Encode a set_external_gps struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_external_gps C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_external_gps_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_external_gps_t* set_external_gps)
{
	return mavlink_msg_set_external_gps_pack_chan(system_id, component_id, chan, msg, set_external_gps->gps_status, set_external_gps->num_sats, set_external_gps->hdop, set_external_gps->time_usec, set_external_gps->lat, set_external_gps->lon, set_external_gps->alt, set_external_gps->vx, set_external_gps->vy, set_external_gps->vz);
}

/**
 * @brief Send a set_external_gps message
 * @param chan MAVLink channel to send the message
 *
 * @param gps_status GPS Status
 * @param num_sats Number of satellites
 * @param hdop HDOP of GPS
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_external_gps_send(mavlink_channel_t chan, uint8_t gps_status, uint8_t num_sats, uint16_t hdop, uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, hdop);
	_mav_put_int16_t(buf, 22, vx);
	_mav_put_int16_t(buf, 24, vy);
	_mav_put_int16_t(buf, 26, vz);
	_mav_put_uint8_t(buf, 28, gps_status);
	_mav_put_uint8_t(buf, 29, num_sats);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, buf, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, buf, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
#else
	mavlink_set_external_gps_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.hdop = hdop;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.gps_status = gps_status;
	packet.num_sats = num_sats;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, (const char *)&packet, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, (const char *)&packet, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_external_gps_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gps_status, uint8_t num_sats, uint16_t hdop, uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, hdop);
	_mav_put_int16_t(buf, 22, vx);
	_mav_put_int16_t(buf, 24, vy);
	_mav_put_int16_t(buf, 26, vz);
	_mav_put_uint8_t(buf, 28, gps_status);
	_mav_put_uint8_t(buf, 29, num_sats);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, buf, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, buf, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
#else
	mavlink_set_external_gps_t *packet = (mavlink_set_external_gps_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->hdop = hdop;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->gps_status = gps_status;
	packet->num_sats = num_sats;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, (const char *)packet, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_EXTERNAL_GPS, (const char *)packet, MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_EXTERNAL_GPS UNPACKING


/**
 * @brief Get field gps_status from set_external_gps message
 *
 * @return GPS Status
 */
static inline uint8_t mavlink_msg_set_external_gps_get_gps_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field num_sats from set_external_gps message
 *
 * @return Number of satellites
 */
static inline uint8_t mavlink_msg_set_external_gps_get_num_sats(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field hdop from set_external_gps message
 *
 * @return HDOP of GPS
 */
static inline uint16_t mavlink_msg_set_external_gps_get_hdop(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field time_usec from set_external_gps message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_set_external_gps_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from set_external_gps message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_set_external_gps_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from set_external_gps message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_set_external_gps_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from set_external_gps message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_set_external_gps_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vx from set_external_gps message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_set_external_gps_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vy from set_external_gps message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_set_external_gps_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field vz from set_external_gps message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_set_external_gps_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Decode a set_external_gps message into a struct
 *
 * @param msg The message to decode
 * @param set_external_gps C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_external_gps_decode(const mavlink_message_t* msg, mavlink_set_external_gps_t* set_external_gps)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_external_gps->time_usec = mavlink_msg_set_external_gps_get_time_usec(msg);
	set_external_gps->lat = mavlink_msg_set_external_gps_get_lat(msg);
	set_external_gps->lon = mavlink_msg_set_external_gps_get_lon(msg);
	set_external_gps->alt = mavlink_msg_set_external_gps_get_alt(msg);
	set_external_gps->hdop = mavlink_msg_set_external_gps_get_hdop(msg);
	set_external_gps->vx = mavlink_msg_set_external_gps_get_vx(msg);
	set_external_gps->vy = mavlink_msg_set_external_gps_get_vy(msg);
	set_external_gps->vz = mavlink_msg_set_external_gps_get_vz(msg);
	set_external_gps->gps_status = mavlink_msg_set_external_gps_get_gps_status(msg);
	set_external_gps->num_sats = mavlink_msg_set_external_gps_get_num_sats(msg);
#else
	memcpy(set_external_gps, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_EXTERNAL_GPS_LEN);
#endif
}
