#ifndef DRONE_MAV_CONNECTION_H
#define DRONE_MAV_CONNECTION_H

#include <iostream>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Include MAVLink headers
#include <mavlink/v2.0/common/mavlink.h>

class MAVLinkConnection {
private:
    int sock;
    struct sockaddr_in target_addr;
    socklen_t target_addr_len;
    bool target_addr_set;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t target_system;
    uint8_t target_component;

public:
    MAVLinkConnection(const char* ip, int port);
    ~MAVLinkConnection();
    
    bool wait_heartbeat();
    bool send_message(mavlink_message_t& msg);
    bool wait_command_ack(uint16_t command, int timeout_sec = 3);
    bool arm_vehicle();
    bool disarm_vehicle();
    bool set_mode(const char* mode_name);
    void send_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons = 0);
    uint8_t get_target_system() const { return target_system; }
};

#endif // DRONE_MAV_CONNECTION_H
