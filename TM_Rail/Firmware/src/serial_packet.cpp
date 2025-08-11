#include "serial_packet.hpp"
#include <ErriezCRC32.h>

/**
 * @brief Send packet to serial
 * 
 * @param type      type of packet
 * @param packet    packet to send
 */
void send_packet(packet_t *packet) {
    // create crc without crc part
    packet->crc = crc32Buffer(packet, sizeof(*packet) - sizeof(packet->crc));
    Serial.write((uint8_t*)packet, sizeof(*packet));
}


extern void handle_packet(packet_t packet);

/**
 * @brief Receive packet from serial, include auto report and crc check
 * 
 */
void handle_serial() {
    const uint8_t identify[4] = {0xAA, 0xCC, 0xFF, 0x99};
    static uint8_t buffer[sizeof(packet_t)] = {0};
    static size_t cursor = 0;
    if(Serial.available()) {
        uint8_t b = Serial.read();
        buffer[cursor] = b;
        // try match identify
        if(cursor < sizeof(identify)) {
            // identify matched
            if(buffer[cursor] == identify[cursor]) {
                cursor++;
            // identify not matched, reset cursor
            } else {
                cursor = 0;
            }
        // size match packet, check crc
        }else if(cursor == sizeof(packet_t)-1) {
            packet_t *packet = (packet_t*)buffer;
            // crc matched
            if(packet->crc == crc32Buffer(packet, sizeof(*packet) - sizeof(packet->crc))) {
                // send report
                packet_t report = CREATE_PACKET(packet->type);
                send_packet(&report);
                // process packet
                handle_packet(*packet);
            }
            cursor = 0;
        // receive data and crc
        }else {
            cursor ++;
        }
    }
}