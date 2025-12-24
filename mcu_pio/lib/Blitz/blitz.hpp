// lib/PacketUtils/include/packet_utils.hpp
#pragma once
#include <stdint.h>
#include <vector>
#include <Arduino.h>
#include <iostream>
#include <cstring>  
#include <stdexcept>
#include "blitz_interfaces.hpp"

// PARSE OR PACK DATA PACKETS
// ^^^^^^^^^^^^^^^^^^^^^^^^^^


// pass the payload(byte packet), from receive_data function , parse it in the form of template interface
template<typename T_parse>
T_parse parse_struct(std::vector<uint8_t>& payload) {
    T_parse result;
    payload.erase(payload.begin());
    std::memcpy(&result, payload.data(), sizeof(T_parse));
    return result;
}

// pass the template interface, it will pack the data in the form of the payload(byte packet)
template<typename T_pack>
std::vector<uint8_t> pack_data(T_pack data, uint8_t id) {
    size_t total_bytes = sizeof(T_pack);
    std::vector<uint8_t> buffer(total_bytes);
    std::memcpy(buffer.data(), &data, total_bytes);

    buffer.insert(buffer.begin(), id);
    return buffer;
}

// SEND OR RECEIVE DATA PACKETS 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// pass the payload, it will write the data to the serial port
void send_data(const std::vector<uint8_t>& buffer) {
    if (Serial) {
        uint8_t header = 0xAA;  
        Serial.write(&header, 1);                 // send header first
        Serial.write(buffer.data(), buffer.size()); // send payload
    }
}

// reveives data and returns the buffer based on the id
std::vector<uint8_t> receive_data() {
    static std::vector<uint8_t> buffer;

    // read all available bytes from Serial
    while (Serial.available()) {
        buffer.push_back(Serial.read());
    }

    // looping until we can return a full packet or buffer is empty
    while (buffer.size() >= 2) { // need at least header + ID
        // find the header 0xAA
        if (buffer[0] != 0xAA) {
            // header not at start, remove the first byte and continue
            buffer.erase(buffer.begin());
            continue;
        }

        // check ID
        uint8_t id = buffer[1];
        size_t expected_size = get_packet_size(id);
        if (expected_size == 0) {
            // unkown id
            buffer.erase(buffer.begin(), buffer.begin() + 1);
            continue;
        }

        // check if full data is available
        if (buffer.size() >= 2 + expected_size) {
            // slice for id + data
            std::vector<uint8_t> packet(buffer.begin() + 1, buffer.begin() + 1 + expected_size);
            // remove consumed bytes from buffer
            buffer.erase(buffer.begin(), buffer.begin() + 1 + expected_size);
            return packet;
        } else {
            // wait for more bytes
            break;
        }
    }

    // No full packet yet
    return {};
}

void debug_state(const std::string& data){

    uint8_t len = static_cast<uint8_t>(data.size());

    // Create a byte buffer: 1 byte for length + string data
    std::vector<uint8_t> buffer;
    buffer.reserve(len + 2);
    uint8_t id = 99;
    buffer.push_back(id);
    buffer.push_back(len);
    buffer.insert(buffer.end(), data.begin(), data.end());

    send_data(buffer);
}

// {header id len string}