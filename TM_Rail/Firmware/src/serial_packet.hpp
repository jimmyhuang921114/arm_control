#ifndef __SERIAL_PACKET_HPP__
#define __SERIAL_PACKET_HPP__

#include <Arduino.h>

/*
    ===== [rx] ===== 
    should report same type after received
    HOME:   no param
    GOTO:   { postiton(0.1mm), velocity(0.1mm/s) }
    MOVE:   { distance(0.1mm), velocity(0.1mm/s)     }
    BAG:    { release/lock(0/1) }
    ===== [tx] =====
    STATUS: { position(0.1mm), velocity(0.1mm/s), homed(0/1), busy(0/1), bag_detect(0/1) }
*/
typedef enum {
	TYPE_HOME,
	TYPE_GOTO,
	TYPE_MOVE,
	TYPE_BAG,
    TYPE_STATUS,
    TYPE_NAME,
}packet_type_t;

typedef struct {
	uint8_t identify[4];	// 0xAACCFF99
	int16_t type;	        // packet_type_t
	int16_t data[5];	
	uint32_t crc;
}packet_t;

#define CREATE_PACKET(p_type) ((packet_t){.identify={0xAA,0xCC,0xFF,0x99},.type=(p_type)})

void send_packet(packet_t *packet);

void handle_serial();

#endif // __SERIAL_PACKET_HPP__