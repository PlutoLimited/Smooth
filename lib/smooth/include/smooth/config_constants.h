#pragma once

#ifndef ESP_PLATFORM

// Values used when compiling Smooth for the host system.
const int CONFIG_SMOOTH_MAX_MQTT_MESSAGE_SIZE = 512;
const int CONFIG_SMOOTH_MAX_MQTT_OUTGOING_MESSAGES = 10;
const int SMOOTH_MQTT_LOGGING_LEVEL = 1;
const int CONFIG_SMOOTH_SOCKET_DISPATCHER_STACK_SIZE = 20480;
const int CONFIG_SMOOTH_TIMER_SERVICE_STACK_SIZE = 3072;
const int CONFIG_LWIP_MAX_SOCKETS = 10;
#endif
