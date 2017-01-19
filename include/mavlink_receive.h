#pragma once

//static int mode = MAV_MODE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */
//static int packet_drops = 0;

//This will contain functions to receive and parse mavlink messages
//and should also hanlde any unsupported commands

static void communication_receive(void);
