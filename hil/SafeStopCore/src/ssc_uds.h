// Minimal UDS (ISO 14229) responder for the bench's two boards (v2.23), over ISO-TP single frames (ISO 15765-2).
// It answers only what a bench-health tester needs; it touches no safety state. Hardware-independent: the same header
// is compiled for the ESP32-S3 and, through ssc_host_api.cpp, for the PC where the tests run it.
//
//   addressing  physical: B request 0x7E2 -> response 0x7EA, A request 0x7E3 -> response 0x7EB; functional: 0x7DF (all nodes)
//   0x3E 00 / 0x3E 80   TesterPresent (80 = suppress the positive response)
//   0x22 F195           software version, ASCII (up to 4 characters, e.g. "2.7")
//   0x22 F18C           ECU serial number, 4 bytes (the low 32 bits of the chip's MAC)
//   anything else       negative response 0x7F <sid> <nrc>: 0x11 service not supported, 0x12 sub-function not supported,
//                       0x13 incorrect length, 0x31 request out of range. A functional request that would get 0x11, 0x12
//                       or 0x31 is not answered at all (ISO 14229: a node stays silent for what it does not offer).
// Multi-frame ISO-TP is deliberately not implemented: every answer fits in one frame.
#pragma once
#include <stdint.h>
#include <string.h>

namespace ssc {
namespace uds {

const uint16_t ID_FUNCTIONAL = 0x7DF;
const uint16_t ID_B_REQ = 0x7E2, ID_B_RESP = 0x7EA, ID_A_REQ = 0x7E3, ID_A_RESP = 0x7EB;
const uint16_t DID_SW_VERSION = 0xF195, DID_SERIAL = 0xF18C;

struct Node {
  uint16_t req_id, resp_id;
  const char *version;   // ASCII, at most 4 characters are sent
  uint32_t serial;
};

inline uint8_t negative(uint8_t sid, uint8_t nrc, bool functional, uint8_t out[8]) {
  if (functional && (nrc == 0x11 || nrc == 0x12 || nrc == 0x31)) return 0;
  out[0] = 0x03; out[1] = 0x7F; out[2] = sid; out[3] = nrc;
  return 4;
}

// rx_id / d / n: the received CAN frame. Returns the number of bytes to send on node.resp_id (0 = no response).
inline uint8_t respond(const Node &node, uint16_t rx_id, const uint8_t *d, uint8_t n, uint8_t out[8]) {
  bool functional = rx_id == ID_FUNCTIONAL;
  if (!functional && rx_id != node.req_id) return 0;
  if (n < 2 || (d[0] & 0xF0) != 0x00) return 0;   // not a single frame
  uint8_t len = d[0] & 0x0F;
  if (len < 1 || len > 7 || len + 1 > n) return 0;
  uint8_t sid = d[1];
  if (sid == 0x3E) {
    if (len != 2) return negative(sid, 0x13, functional, out);
    if ((d[2] & 0x7F) != 0x00) return negative(sid, 0x12, functional, out);
    if (d[2] & 0x80) return 0;   // suppressPosRspMsgIndicationBit
    out[0] = 0x02; out[1] = 0x7E; out[2] = 0x00;
    return 3;
  }
  if (sid == 0x22) {
    if (len != 3) return negative(sid, 0x13, functional, out);
    uint16_t did = (uint16_t)(d[2] << 8 | d[3]);
    if (did == DID_SW_VERSION) {
      uint8_t k = (uint8_t)strnlen(node.version, 4);
      out[0] = (uint8_t)(3 + k); out[1] = 0x62; out[2] = d[2]; out[3] = d[3];
      memcpy(out + 4, node.version, k);
      return (uint8_t)(4 + k);
    }
    if (did == DID_SERIAL) {
      out[0] = 0x07; out[1] = 0x62; out[2] = d[2]; out[3] = d[3];
      out[4] = (uint8_t)(node.serial >> 24); out[5] = (uint8_t)(node.serial >> 16);
      out[6] = (uint8_t)(node.serial >> 8); out[7] = (uint8_t)node.serial;
      return 8;
    }
    return negative(sid, 0x31, functional, out);
  }
  return negative(sid, 0x11, functional, out);
}

}  // namespace uds
}  // namespace ssc
