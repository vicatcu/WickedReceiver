/*
 * WickedReceiver.h
 *
 *      Created on: Dec 26, 2010
 *          Author: Victor Aprea
 *   Documentation: http://node.wickeddevice.com
 *
 * Licensed under Creative Commons Attribution-Noncommercial-Share Alike 3.0
 *
 *        Revision: 639
 *
 */

#ifndef WickedReceiver_H_
#define WickedReceiver_H_

#include <stdint.h>
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#define STATE_WAITING_FOR_START_BYTE 0
#define STATE_RECEIVED_START_BYTE    1
#define STATE_RECEIVING_PACKET       2

#define NUM_BYTES_IN_DECODED_PACKET  8
#define NUM_BYTES_IN_ENCODED_PACKET  48

#define PACKET_AGE_OUT_MS    1000
#define NUM_DECODES_REQUIRED 2
#define NUM_RETRANSMITS 4
#define NUM_RETRANSMITS_PLUS_1 (NUM_RETRANSMITS + 1)

#define DECODE_AMBIGUITY_TOLERANCE   2

#ifndef RADIO_BAUD_RATE
#define RADIO_BAUD_RATE 2400
#endif

#define STATISTICS_LAG_FACTOR 1.5
#define STATISTICS_TIME_WINDOW_MS   ((long) (NUM_RETRANSMITS * 1000.0 * NUM_BYTES_IN_ENCODED_PACKET * 100.0 * STATISTICS_LAG_FACTOR / RADIO_BAUD_RATE)) // effectively how long did it take to transmit the data
#define STATISTICS_SILENT_WINDOW_MS 1500  // how long must you perceive silence before assuming you are between bursts

typedef struct{
  long     timestamp;
  uint8_t  packet[NUM_BYTES_IN_DECODED_PACKET];
  uint8_t  num_occurences;
}
stat_record;

class WickedReceiver{
public:
	WickedReceiver(uint8_t node_id_filter);
	WickedReceiver(void);
    // primary methods
    uint8_t  processEncodedByte(uint8_t incoming_byte);
    void     nextPacket(void);    
    
    // convenience accessors
    uint8_t  getDecodedNodeId(void);
    uint16_t getDecodedPacketNumber(void);
    uint8_t  getDecodedSensor1(void);
    uint8_t  getDecodedSensor2(void);
    uint8_t  getDecodedSensor3(void);
    uint8_t  getDecodedDigitalCount(void);
    uint8_t  getDecodedByte(uint8_t byte_number); // for random access    
    uint8_t  getEncodedByte(uint8_t byte_number); 
    
    // utility statistics functions
    uint32_t getMillisSinceLastDecode(void);
    uint8_t  inSilenceWindow(void);

    uint8_t candidate_decode_multiplicity[NUM_BYTES_IN_DECODED_PACKET * 2];
    uint8_t candidate_decode_distance[NUM_BYTES_IN_DECODED_PACKET * 2];

    stat_record last_N_ids[NUM_RETRANSMITS_PLUS_1];
private:
    void decoder(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5,  uint8_t decode_index);
    void dectrans(uint32_t index, uint8_t decode_index);
    void init(void);

    static const uint8_t  enctable[16];
    static const uint32_t dectable[16];

    uint8_t  candidate_decoded_messages[NUM_BYTES_IN_DECODED_PACKET * 2][DECODE_AMBIGUITY_TOLERANCE];
    // first dimension is index by nibble
    // second dimension is candidate_value_index

    void aggressive_candidate_decode(void);
    
    uint8_t state;
    uint8_t packet_ready;
    uint8_t node_id_filter;
    uint8_t promiscuousMode;

    uint8_t writeIndex;
    uint8_t inbyte[NUM_BYTES_IN_ENCODED_PACKET];
    uint8_t num_bytes_until_decode_is_possible;

    uint8_t decoded_data[NUM_BYTES_IN_DECODED_PACKET];

    uint8_t retx_id_buffer_index;

    uint32_t last_packet_id;
    uint16_t last_node_id;
    uint32_t timestamp_of_last_decode;

    uint8_t computeChecksum(void);
    uint8_t crc8( uint8_t inCrc, uint8_t inData );
    uint8_t register_packet_in_history(uint32_t current_time);
    uint8_t computeHammingDistance(uint32_t a, uint32_t b);
    uint8_t findMinimumDistanceHammingCode(uint32_t encoded_value, uint8_t decode_index);
};

#endif /* WickedReceiver_H_ */
