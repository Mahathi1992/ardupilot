/** @file
 *    @brief MAVLink comm protocol testsuite generated from zmotion.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ZMOTION_TESTSUITE_H
#define ZMOTION_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_zmotion(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_zmotion(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_zas_gimbal_zoom_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zas_gimbal_zoom_cmd_t packet_in = {
        17235,17339,17443,17547,17651,163,230,41,108
    };
    mavlink_zas_gimbal_zoom_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Zoom = packet_in.Zoom;
        packet1.ZoomRate = packet_in.ZoomRate;
        packet1.trk_x = packet_in.trk_x;
        packet1.trk_y = packet_in.trk_y;
        packet1.offset_box = packet_in.offset_box;
        packet1.trk_cmd = packet_in.trk_cmd;
        packet1.offset_x = packet_in.offset_x;
        packet1.offset_y = packet_in.offset_y;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_zoom_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zas_gimbal_zoom_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_zoom_cmd_pack(system_id, component_id, &msg , packet1.Zoom , packet1.ZoomRate , packet1.trk_x , packet1.trk_y , packet1.trk_cmd , packet1.offset_x , packet1.offset_y , packet1.offset_box , packet1.mode );
    mavlink_msg_zas_gimbal_zoom_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_zoom_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Zoom , packet1.ZoomRate , packet1.trk_x , packet1.trk_y , packet1.trk_cmd , packet1.offset_x , packet1.offset_y , packet1.offset_box , packet1.mode );
    mavlink_msg_zas_gimbal_zoom_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zas_gimbal_zoom_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_zoom_cmd_send(MAVLINK_COMM_1 , packet1.Zoom , packet1.ZoomRate , packet1.trk_x , packet1.trk_y , packet1.trk_cmd , packet1.offset_x , packet1.offset_y , packet1.offset_box , packet1.mode );
    mavlink_msg_zas_gimbal_zoom_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZAS_GIMBAL_ZOOM_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD) != NULL);
#endif
}

static void mavlink_test_zas_gimbal_usr_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zas_gimbal_usr_cmd_t packet_in = {
        5,72
    };
    mavlink_zas_gimbal_usr_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.calib_stat = packet_in.calib_stat;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zas_gimbal_usr_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_pack(system_id, component_id, &msg , packet1.calib_stat , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.calib_stat , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zas_gimbal_usr_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_send(MAVLINK_COMM_1 , packet1.calib_stat , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZAS_GIMBAL_USR_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD) != NULL);
#endif
}

static void mavlink_test_zas_gimbal_usr_cmd_pilot_stab(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,65,132,199
    };
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.yawrate = packet_in.yawrate;
        packet1.pitchrate = packet_in.pitchrate;
        packet1.rollrate = packet_in.rollrate;
        packet1.pan = packet_in.pan;
        packet1.tilt = packet_in.tilt;
        packet1.calib_stat = packet_in.calib_stat;
        packet1.submode = packet_in.submode;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_pack(system_id, component_id, &msg , packet1.yawrate , packet1.pitchrate , packet1.rollrate , packet1.pan , packet1.tilt , packet1.calib_stat , packet1.submode , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.yawrate , packet1.pitchrate , packet1.rollrate , packet1.pan , packet1.tilt , packet1.calib_stat , packet1.submode , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_send(MAVLINK_COMM_1 , packet1.yawrate , packet1.pitchrate , packet1.rollrate , packet1.pan , packet1.tilt , packet1.calib_stat , packet1.submode , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZAS_GIMBAL_USR_CMD_PILOT_STAB") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB) != NULL);
#endif
}

static void mavlink_test_zas_gimbal_usr_cmd_geopoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zas_gimbal_usr_cmd_geopoint_t packet_in = {
        963497464,963497672,963497880,41,108
    };
    mavlink_zas_gimbal_usr_cmd_geopoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_latitude = packet_in.target_latitude;
        packet1.target_longitude = packet_in.target_longitude;
        packet1.target_altitude = packet_in.target_altitude;
        packet1.calib_stat = packet_in.calib_stat;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_pack(system_id, component_id, &msg , packet1.target_latitude , packet1.target_longitude , packet1.target_altitude , packet1.calib_stat , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_latitude , packet1.target_longitude , packet1.target_altitude , packet1.calib_stat , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_send(MAVLINK_COMM_1 , packet1.target_latitude , packet1.target_longitude , packet1.target_altitude , packet1.calib_stat , packet1.mode );
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZAS_GIMBAL_USR_CMD_GEOPOINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT) != NULL);
#endif
}

static void mavlink_test_zas_gimbal_status_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zas_gimbal_status_msg_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,18483,18587,89,156,223,34,101
    };
    mavlink_zas_gimbal_status_msg_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.cmd_pan = packet_in.cmd_pan;
        packet1.cmd_tilt = packet_in.cmd_tilt;
        packet1.pan = packet_in.pan;
        packet1.tilt = packet_in.tilt;
        packet1.elevation = packet_in.elevation;
        packet1.azimuth = packet_in.azimuth;
        packet1.trk_x = packet_in.trk_x;
        packet1.trk_y = packet_in.trk_y;
        packet1.Zoom = packet_in.Zoom;
        packet1.wb_mode = packet_in.wb_mode;
        packet1.mode = packet_in.mode;
        packet1.cam_type = packet_in.cam_type;
        packet1.trk_status = packet_in.trk_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_status_msg_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zas_gimbal_status_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_status_msg_pack(system_id, component_id, &msg , packet1.trk_x , packet1.trk_y , packet1.cmd_pan , packet1.cmd_tilt , packet1.pan , packet1.tilt , packet1.elevation , packet1.azimuth , packet1.Zoom , packet1.wb_mode , packet1.mode , packet1.cam_type , packet1.trk_status );
    mavlink_msg_zas_gimbal_status_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_status_msg_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.trk_x , packet1.trk_y , packet1.cmd_pan , packet1.cmd_tilt , packet1.pan , packet1.tilt , packet1.elevation , packet1.azimuth , packet1.Zoom , packet1.wb_mode , packet1.mode , packet1.cam_type , packet1.trk_status );
    mavlink_msg_zas_gimbal_status_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zas_gimbal_status_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zas_gimbal_status_msg_send(MAVLINK_COMM_1 , packet1.trk_x , packet1.trk_y , packet1.cmd_pan , packet1.cmd_tilt , packet1.pan , packet1.tilt , packet1.elevation , packet1.azimuth , packet1.Zoom , packet1.wb_mode , packet1.mode , packet1.cam_type , packet1.trk_status );
    mavlink_msg_zas_gimbal_status_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZAS_GIMBAL_STATUS_MSG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG) != NULL);
#endif
}

static void mavlink_test_zmotion_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZMOTION_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zmotion_command_t packet_in = {
        963497464,963497672,963497880,963498088,18067,187,254
    };
    mavlink_zmotion_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.custom_param1 = packet_in.custom_param1;
        packet1.custom_param2 = packet_in.custom_param2;
        packet1.custom_param3 = packet_in.custom_param3;
        packet1.custom_param4 = packet_in.custom_param4;
        packet1.seq_identifier = packet_in.seq_identifier;
        packet1.zmotion_cmd_type = packet_in.zmotion_cmd_type;
        packet1.actual_cmd = packet_in.actual_cmd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zmotion_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_pack(system_id, component_id, &msg , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.custom_param1 , packet1.custom_param2 , packet1.custom_param3 , packet1.custom_param4 );
    mavlink_msg_zmotion_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.custom_param1 , packet1.custom_param2 , packet1.custom_param3 , packet1.custom_param4 );
    mavlink_msg_zmotion_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zmotion_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_send(MAVLINK_COMM_1 , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.custom_param1 , packet1.custom_param2 , packet1.custom_param3 , packet1.custom_param4 );
    mavlink_msg_zmotion_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZMOTION_COMMAND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZMOTION_COMMAND) != NULL);
#endif
}

static void mavlink_test_zmotion_command_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZMOTION_COMMAND_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zmotion_command_response_t packet_in = {
        963497464,963497672,963497880,963498088,18067,187,254
    };
    mavlink_zmotion_command_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.response1 = packet_in.response1;
        packet1.response2 = packet_in.response2;
        packet1.response3 = packet_in.response3;
        packet1.response4 = packet_in.response4;
        packet1.seq_identifier = packet_in.seq_identifier;
        packet1.zmotion_cmd_type = packet_in.zmotion_cmd_type;
        packet1.actual_cmd = packet_in.actual_cmd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZMOTION_COMMAND_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZMOTION_COMMAND_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zmotion_command_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_response_pack(system_id, component_id, &msg , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.response1 , packet1.response2 , packet1.response3 , packet1.response4 );
    mavlink_msg_zmotion_command_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.response1 , packet1.response2 , packet1.response3 , packet1.response4 );
    mavlink_msg_zmotion_command_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zmotion_command_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_command_response_send(MAVLINK_COMM_1 , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.response1 , packet1.response2 , packet1.response3 , packet1.response4 );
    mavlink_msg_zmotion_command_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZMOTION_COMMAND_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZMOTION_COMMAND_RESPONSE) != NULL);
#endif
}

static void mavlink_test_zmotion_payload2_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_zmotion_payload2_response_t packet_in = {
        17235,139,206,17,84,151,218
    };
    mavlink_zmotion_payload2_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq_identifier = packet_in.seq_identifier;
        packet1.zmotion_cmd_type = packet_in.zmotion_cmd_type;
        packet1.actual_cmd = packet_in.actual_cmd;
        packet1.response1 = packet_in.response1;
        packet1.response2 = packet_in.response2;
        packet1.response3 = packet_in.response3;
        packet1.response4 = packet_in.response4;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_payload2_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_zmotion_payload2_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_payload2_response_pack(system_id, component_id, &msg , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.response1 , packet1.response2 , packet1.response3 , packet1.response4 );
    mavlink_msg_zmotion_payload2_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_payload2_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.response1 , packet1.response2 , packet1.response3 , packet1.response4 );
    mavlink_msg_zmotion_payload2_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_zmotion_payload2_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_zmotion_payload2_response_send(MAVLINK_COMM_1 , packet1.zmotion_cmd_type , packet1.actual_cmd , packet1.seq_identifier , packet1.response1 , packet1.response2 , packet1.response3 , packet1.response4 );
    mavlink_msg_zmotion_payload2_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ZMOTION_PAYLOAD2_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE) != NULL);
#endif
}

static void mavlink_test_zmotion(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_zas_gimbal_zoom_cmd(system_id, component_id, last_msg);
    mavlink_test_zas_gimbal_usr_cmd(system_id, component_id, last_msg);
    mavlink_test_zas_gimbal_usr_cmd_pilot_stab(system_id, component_id, last_msg);
    mavlink_test_zas_gimbal_usr_cmd_geopoint(system_id, component_id, last_msg);
    mavlink_test_zas_gimbal_status_msg(system_id, component_id, last_msg);
    mavlink_test_zmotion_command(system_id, component_id, last_msg);
    mavlink_test_zmotion_command_response(system_id, component_id, last_msg);
    mavlink_test_zmotion_payload2_response(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ZMOTION_TESTSUITE_H
