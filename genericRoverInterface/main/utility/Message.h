
#pragma once

#include <iostream>
#include <vector>

extern "C" {

    using vect8 = std::vector<uint8_t>;
    using vectIter = std::vector<uint8_t>::const_iterator;

    class Message {
    public:
        uint8_t src;                     // sender id
        uint8_t dst;                     // destination id (HOST_ID reserved)
        uint8_t type;                    // user-defined type
        vect8 payload;    // payload bytes (binary)

        Message() {
            this->src = 0;
            this->dst = 0;
            this->type = -1;
            this->payload.clear();
        }

        Message(Message& msg) {
            this->src = msg.src;
            this->dst = msg.dst;
            this->type = msg.type;

            // Deep copy - since it is primitive type
            this->payload = msg.payload;
        }
    
        // Frame constants
        static constexpr uint8_t START_BYTE = 0x7E;
        static constexpr uint8_t HOST_ID = 0xFF; // reserved ID for host (RasPi)

        bool isempty() {
            return (payload.size() == 0) ? true:false;
        }

        void clear() {
            src = 0;
            dst = 0;
            type = -1;
            payload.clear();
        }

        // Serialize into wire frame:
        // [START][LEN][SRC][DST][TYPE][PAYLOAD...][CHK]
        // LEN = number of bytes after LEN (SRC..CHK)
        vect8 toFrame() const {
            vect8 frame;
            frame.push_back(START_BYTE);
            uint8_t len = static_cast<uint8_t>(3 + payload.size() + 1); // SRC,DST,TYPE + payload + CHK
            frame.push_back(len);
            frame.push_back(src);
            frame.push_back(dst);
            frame.push_back(type);
            frame.insert(frame.end(), payload.begin(), payload.end());
            uint8_t chk = checksum(frame.begin() + 2, frame.end()); // compute over SRC..end (without START,LEN)
            frame.push_back(chk);
            return frame;
        }

        // Parse a frame (full frame including START & LEN & CHK). Returns true on success.
        static bool fromFrame(const vect8& frame, Message& out) {
            
            // minimum frame length
            if (frame.size() < 6) 
                return false; 
            
            // Check START byte
            if (frame[0] != START_BYTE) 
                return false;
            
            // Check length
            uint8_t len = frame[1];
            if (len + 2 != frame.size()) 
                return false;

            // Compute and verify checksum
            uint8_t chk = checksum(frame.begin() + 2, frame.end() - 1);
            if (chk != frame.back()) 
                return false;
            
            // Populate message metadata and payload
            out.src = frame[2];
            out.dst = frame[3];
            out.type = frame[4];
            size_t payloadLen = static_cast<size_t>(len) - 4; // SRC,DST,TYPE,CHK
            out.payload.clear();
            if (payloadLen > 0) {
                out.payload.insert(out.payload.end(), frame.begin() + 5, frame.begin() + 5 + payloadLen);
            }
            return true;
        }

        // Assignment operator
        Message operator=(Message& msg) {
            this->src = msg.src;
            this->dst = msg.dst;
            this->type = msg.type;

            // Deep copy - since it is primitive type
            this->payload = msg.payload;
        }

        // small helper to compute simple XOR checksum
        static uint8_t checksum(vectIter begin, vectIter end) {
            uint8_t acc = 0;
            for (auto it = begin; it != end; ++it) acc ^= *it;
            return acc;
        }
    };
}