/**
 * Implementation of MAVLinkConnection class for ArduPilot SITL communication.
 */

#include "gps_denied_nav/drone_mav_connection.hpp"

MAVLinkConnection::MAVLinkConnection(const char* ip, int port) 
    : system_id(255), component_id(0), target_addr_set(false) {
        // Create UDP socket
        sock = socket(PF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        // Set socket timeout to avoid blocking forever
        struct timeval tv;
        tv.tv_sec = 0;  // 0 second timeout
        tv.tv_usec = 100000;
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            throw std::runtime_error("Failed to set socket timeout");
        }

        // Setup target address length
        target_addr_len = sizeof(target_addr);
        memset(&target_addr, 0, target_addr_len);

        // Bind to local port for receiving
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(port); // Local port
        local_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            throw std::runtime_error("Failed to bind socket");
        }
    }

MAVLinkConnection::~MAVLinkConnection() {
    close(sock);
}

bool MAVLinkConnection::wait_heartbeat() {
        
        uint8_t buf[2048];
        mavlink_message_t msg;
        mavlink_status_t status;

        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
            ssize_t len = recvfrom(sock, buf, sizeof(buf), 0, 
                                  (struct sockaddr*)&target_addr, &target_addr_len);
            if (len > 0) {
                // We received data, now we know where to send to
                target_addr_set = true;
                
                for (ssize_t i = 0; i < len; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            target_system = msg.sysid;
                            target_component = msg.compid;
                            
                            // Print the address we'll send to
                            char ip_str[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &(target_addr.sin_addr), ip_str, INET_ADDRSTRLEN);

                            return true;
                        }
                    }
                }
            }
        }
        return false;
}

bool MAVLinkConnection::send_message(mavlink_message_t& msg) {
        if (!target_addr_set) {
            return false;
        }
        
        uint8_t buf[2048];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        
        ssize_t sent = sendto(sock, buf, len, 0, 
                             (struct sockaddr*)&target_addr, target_addr_len);
        return sent == len;
}

bool MAVLinkConnection::wait_command_ack(uint16_t command, int timeout_sec) {
        uint8_t buf[2048];
        mavlink_message_t msg;
        mavlink_status_t status;

        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout_sec)) {
            ssize_t len = recvfrom(sock, buf, sizeof(buf), 0, 
                                  (struct sockaddr*)&target_addr, &target_addr_len);
            if (len > 0) {
                for (ssize_t i = 0; i < len; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);
                            if (ack.command == command) {
                                return ack.result == MAV_RESULT_ACCEPTED;
                            }
                        }
                    }
                }
            }
        }
        return false;
}

bool MAVLinkConnection::arm_vehicle() {
        mavlink_message_t msg;
        mavlink_command_long_t cmd = {};
        
        cmd.target_system = target_system;
        cmd.target_component = target_component;
        cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
        cmd.confirmation = 0;
        cmd.param1 = 1.0f; // 1 to arm
        
        mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
        send_message(msg);
        
        if (wait_command_ack(MAV_CMD_COMPONENT_ARM_DISARM)) {
            return true;
        } else {
            return false;
        }
}

bool MAVLinkConnection::disarm_vehicle() {
    mavlink_message_t msg;
    mavlink_command_long_t cmd = {};
    
    cmd.target_system = target_system;
    cmd.target_component = target_component;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 0.0f; // 0 to disarm
    
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
    send_message(msg);
    
    if (wait_command_ack(MAV_CMD_COMPONENT_ARM_DISARM)) {
        return true;
    } else {
        return false;
    }
}

bool MAVLinkConnection::set_mode(const char* mode_name) {
        
        // For simplicity, using STABILIZE mode (mode 0 for copter)
        // In a real implementation, you'd need a mode mapping
        uint32_t custom_mode = 0; // STABILIZE for copter
        
        mavlink_message_t msg;
        mavlink_set_mode_t mode_msg = {};
        
        mode_msg.target_system = target_system;
        mode_msg.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode_msg.custom_mode = custom_mode;
        
        mavlink_msg_set_mode_encode(system_id, component_id, &msg, &mode_msg);
        send_message(msg);
        
        return true;
}

void MAVLinkConnection::send_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons) {
        mavlink_message_t msg;
        mavlink_manual_control_t manual = {};
        
        manual.target = target_system;
        manual.x = x;       // pitch
        manual.y = y;       // roll
        manual.z = z;       // throttle
        manual.r = r;       // yaw
        manual.buttons = buttons;
        
        mavlink_msg_manual_control_encode(system_id, component_id, &msg, &manual);
        send_message(msg);
}
