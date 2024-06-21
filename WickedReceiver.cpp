/*
 * WickedReceiver.cpp
 *
 *      Created on: Dec 26, 2010
 *          Author: Victor Aprea
 *   Documentation: http://node.wickeddevice.com
 *
 * Licensed under Creative Commons Attribution-Noncommercial-Share Alike 3.0
 *
 *        Revision: 501
 *
 */

#include "WickedReceiver.h"
#include <string.h>

const uint8_t WickedReceiver::enctable[16]={0x07,0x0b,0x0d,0x0e,0x13,0x15,0x16,0x19,0x1a,0x1c,0x23,0x25,0x26,0x29,0x2a,0x2c};
const uint32_t WickedReceiver::dectable[16]={
    0xC7711C,
    0xCBB22C,
    0x4DD334,
    0x8EE338,
    0xD3344D,
    0x555555,
    0x966559,
    0x599665,
    0x9AA669,
    0x1CC771,
    0xE3388E,
    0x655996,
    0xA6699A,
    0x699AA6,
    0xAAAAAA,
    0x2CCBB2
};

WickedReceiver::WickedReceiver(uint8_t node_id_filter){
	init();
	promiscuousMode = 0;
	this->node_id_filter = node_id_filter;
}

WickedReceiver::WickedReceiver(void){
	init();
	promiscuousMode = 1;
}

void WickedReceiver::init(void){
	state = STATE_WAITING_FOR_START_BYTE;
	writeIndex = 0;		
	retx_id_buffer_index = 0;
	last_packet_id = (uint32_t) -1;
    last_node_id   = (uint16_t) -1;
	timestamp_of_last_decode = 0;
	packet_ready = 0;

	memset(last_N_ids, 0, NUM_RETRANSMITS_PLUS_1*sizeof(stat_record));

	num_bytes_until_decode_is_possible = NUM_BYTES_IN_ENCODED_PACKET;
}

uint8_t WickedReceiver::processEncodedByte(uint8_t incoming_byte){
  uint8_t return_value = 0;
  uint8_t startIndex   = 0;
  uint8_t num_bytes_decoded = 0;
  uint8_t decode_index      = 0;

  if(num_bytes_until_decode_is_possible > 0){
      num_bytes_until_decode_is_possible--;
  }
  
  // treat every received byte as the last byte of a completed packet
  inbyte[writeIndex] = incoming_byte;
  
  if(num_bytes_until_decode_is_possible == 0){
      startIndex = (writeIndex + 1) % NUM_BYTES_IN_ENCODED_PACKET;

      num_bytes_decoded = 0;
      decode_index = startIndex;

      while(num_bytes_decoded < NUM_BYTES_IN_DECODED_PACKET){
        decoder(
          inbyte[(decode_index+0) % NUM_BYTES_IN_ENCODED_PACKET],
          inbyte[(decode_index+1) % NUM_BYTES_IN_ENCODED_PACKET],
          inbyte[(decode_index+2) % NUM_BYTES_IN_ENCODED_PACKET],
          inbyte[(decode_index+3) % NUM_BYTES_IN_ENCODED_PACKET],
          inbyte[(decode_index+4) % NUM_BYTES_IN_ENCODED_PACKET],
          inbyte[(decode_index+5) % NUM_BYTES_IN_ENCODED_PACKET],
          num_bytes_decoded);

        num_bytes_decoded += 1;
        decode_index      += 6;
      }

      aggressive_candidate_decode();

      uint8_t  node_id   = decoded_data[0];
      uint16_t packet_id = decoded_data[1] + decoded_data[2]*256;
      uint8_t  crc = computeChecksum();
      uint8_t  valid_crc = 0;

      uint8_t num_high_bits_in_node_id = computeHammingDistance(0, node_id); // how many bits are high in the node id
      uint8_t num_occurences = 0;

      if( crc == decoded_data[NUM_BYTES_IN_DECODED_PACKET-1] &&            // a candidate packet has a correct crc and a valid node id
          (num_high_bits_in_node_id == 4)){                                // by design, all valid node-ids have exactly 4-bits high)
        timestamp_of_last_decode = millis();
        valid_crc = 1;
        num_occurences = register_packet_in_history(timestamp_of_last_decode);
      }


      if( (valid_crc == 1)                                                    &&  // the packet has a valid crc and a valid address
          (num_occurences == NUM_DECODES_REQUIRED)                            &&  // once you have seen it exactly enough times
          (packet_ready == 0)                                                 &&  // the user has unlocked the receiver by calling nextPacket()
          (promiscuousMode || (node_id == node_id_filter))){                        // the address filter passes if in non-promiscuous mode
        last_packet_id = packet_id;
        last_node_id   = node_id;
        packet_ready = 1;
        return_value = 1;

        // since we've decoded a valid packet, wait for at least a packet worth of bytes before attempting another decode
        num_bytes_until_decode_is_possible = NUM_BYTES_IN_ENCODED_PACKET;

        // wipe out the history, except for this packet
        for(uint8_t ii = 0; ii < NUM_BYTES_IN_DECODED_PACKET; ii++){
            if(last_N_ids[ii].packet[0] != node_id){
                last_N_ids[ii].num_occurences = 0;
                last_N_ids[ii].packet[0]      = 0xff;
            }
        }
      }
  }
  
  writeIndex = (writeIndex + 1) % NUM_BYTES_IN_ENCODED_PACKET;
  return return_value;
}

void WickedReceiver::decoder(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t decode_index){
  uint32_t index;
  
  index = (((uint32_t) b0) << 16) |
          (((uint32_t) b1) << 8)  |
          (((uint32_t) b2) << 0)  ;

  dectrans(index, 2*decode_index);


  index = (((uint32_t) b3) << 16) |
          (((uint32_t) b4) << 8)  |
          (((uint32_t) b5) << 0)  ;

  dectrans(index, 2*decode_index + 1);
}

void WickedReceiver::dectrans(uint32_t index, uint8_t decode_index){  
  if(decode_index < NUM_BYTES_IN_DECODED_PACKET * 2){
    findMinimumDistanceHammingCode(index, decode_index);  
  }
}

// returns the sum of all the bytes modulo 256 (i.e. ignoring overflow)
uint8_t WickedReceiver::computeChecksum(void){
    uint8_t crc = 0;
    for(uint8_t ii = 0; ii < NUM_BYTES_IN_DECODED_PACKET - 1; ii++){
        crc = crc8( crc, decoded_data[ii] );
    }
    return crc;
}

uint8_t WickedReceiver::crc8( uint8_t inCrc, uint8_t inData ){
    uint8_t ii;
    uint8_t data = inCrc ^ inData;

    for ( ii = 0; ii < 8; ii++ ){
        if ( ( data & 0x80 ) != 0 ){
            data <<= 1;
            data ^= 0x07;
        }
        else{
            data <<= 1;
        }
    }

    return data;
}

uint8_t WickedReceiver::register_packet_in_history(uint32_t current_time){
    uint8_t count = 0;
    uint8_t found = 0;
    uint32_t packet_age_ms = 0;

    //search the history for a perfect packet match
    for(uint8_t ii = 0; ii < NUM_RETRANSMITS_PLUS_1; ii++){
      packet_age_ms = current_time - last_N_ids[ii].timestamp;
      if(packet_age_ms < PACKET_AGE_OUT_MS){ // if the entry has not expired
          found = 1;
          for(uint8_t jj = 0; jj < NUM_BYTES_IN_DECODED_PACKET; jj++){
              if(decoded_data[jj] != last_N_ids[ii].packet[jj]){
                  found = 0;
                  break;
              }
          }

          // if you find one, increment it's occurence count and return the new count
          if(found == 1){
              count = ++(last_N_ids[ii].num_occurences);
              last_N_ids[ii].timestamp = current_time;
              break;
          }
      }
      else{ // mark itas aged out
          last_N_ids[ii].packet[0]      = 0xff;
          last_N_ids[ii].num_occurences = 0;
      }
    }

    // if the packet was not found, add it and advance the buffer index
    if(count == 0){
        for(uint8_t jj = 0; jj < NUM_BYTES_IN_DECODED_PACKET; jj++){
            last_N_ids[retx_id_buffer_index].packet[jj] = decoded_data[jj];
        }
        last_N_ids[retx_id_buffer_index].num_occurences = 1;
        last_N_ids[retx_id_buffer_index].timestamp = current_time;
        count = 1;
        retx_id_buffer_index = (retx_id_buffer_index + 1) % (NUM_RETRANSMITS_PLUS_1);
    }

    return count;
}

// counts the number of different bits between a and b
uint8_t  WickedReceiver::computeHammingDistance(uint32_t a, uint32_t b){
    uint32_t n = a ^ b;
    uint8_t count = 0;
    while (n){
        count++;
        n &= (n - 1);
    }
    return count;
}

// assigns the first encountered "nearest" codeword index into nearest_codeword_index variable
// returns 1 if it was unambiguously the nearest codeword and zero otherwise
uint8_t WickedReceiver::findMinimumDistanceHammingCode(uint32_t encoded_value, uint8_t decode_index){
    uint8_t minHammingDistance = 0xFF;
    uint8_t num_occurences_of_nearest = 1;
    uint8_t overflow_flag = 0;    
    for(uint8_t ii = 0; ii < 16; ii++){
        uint8_t distance = computeHammingDistance(encoded_value, dectable[ii]);
        if( distance < minHammingDistance){
            num_occurences_of_nearest = 1;
            minHammingDistance = distance;
            candidate_decoded_messages[decode_index][0] = ii;
        }
        else if(distance == minHammingDistance){           
            if(num_occurences_of_nearest < DECODE_AMBIGUITY_TOLERANCE){
              candidate_decoded_messages[decode_index][num_occurences_of_nearest] = ii;
              num_occurences_of_nearest++;
            }              
            else{
              overflow_flag = 1;
            }
        }
    }      
    
    candidate_decode_multiplicity[decode_index] = num_occurences_of_nearest;  
    candidate_decode_distance[decode_index] = minHammingDistance;
    
    return overflow_flag;
}

void WickedReceiver::aggressive_candidate_decode(void){    

//    uint8_t _break = 0;
//    uint8_t _index[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//
//    memset(decoded_data, 0, NUM_BYTES_IN_DECODED_PACKET);
//
//    // at this point the candidate_decoded_messages and candidate_decode_multiplicity have all the possible messages
//    // for each nibble, there are up to DECODE_AMBIGUITY_TOLERANCE possible values, try every combination
//    // if a checksum constraint is satisfied, accept that combination as the correct value
//    for(; _index[0x00] < candidate_decode_multiplicity[0x00] && _break == 0; _index[0x00]++){
//    for(; _index[0x01] < candidate_decode_multiplicity[0x01] && _break == 0; _index[0x01]++){
//    for(; _index[0x02] < candidate_decode_multiplicity[0x02] && _break == 0; _index[0x02]++){
//    for(; _index[0x03] < candidate_decode_multiplicity[0x03] && _break == 0; _index[0x03]++){
//    for(; _index[0x04] < candidate_decode_multiplicity[0x04] && _break == 0; _index[0x04]++){
//    for(; _index[0x05] < candidate_decode_multiplicity[0x05] && _break == 0; _index[0x05]++){
//    for(; _index[0x06] < candidate_decode_multiplicity[0x06] && _break == 0; _index[0x06]++){
//    for(; _index[0x07] < candidate_decode_multiplicity[0x07] && _break == 0; _index[0x07]++){
//    for(; _index[0x08] < candidate_decode_multiplicity[0x08] && _break == 0; _index[0x08]++){
//    for(; _index[0x09] < candidate_decode_multiplicity[0x09] && _break == 0; _index[0x09]++){
//    for(; _index[0x0a] < candidate_decode_multiplicity[0x0a] && _break == 0; _index[0x0a]++){
//    for(; _index[0x0b] < candidate_decode_multiplicity[0x0b] && _break == 0; _index[0x0b]++){
//    for(; _index[0x0c] < candidate_decode_multiplicity[0x0c] && _break == 0; _index[0x0c]++){
//    for(; _index[0x0d] < candidate_decode_multiplicity[0x0d] && _break == 0; _index[0x0d]++){
//    for(; _index[0x0e] < candidate_decode_multiplicity[0x0e] && _break == 0; _index[0x0e]++){
//    for(; _index[0x0f] < candidate_decode_multiplicity[0x0f] && _break == 0; _index[0x0f]++){
//        // this place represents a candidate group of nibbles - representing a possible message
//        for(uint8_t ii = 0; ii < 8; ii++){
//            uint8_t even_index = ii * 2;
//            uint8_t odd_index  = even_index + 1;
//            decoded_data[ii] = (candidate_decoded_messages[odd_index][_index[odd_index]] << 4) +
//                    candidate_decoded_messages[even_index][_index[even_index]];
//        }
//
//        if(computeChecksum() == decoded_data[NUM_BYTES_IN_DECODED_PACKET - 1]){
//            // wow, we have found a possible valid message - kick out of this crazy loop
//            _break = 1;
//        }
//    }}}}}}}}}}}}}}}}

	for(uint8_t ii = 0; ii < NUM_BYTES_IN_DECODED_PACKET; ii++){
		uint8_t even_index = ii * 2;
		uint8_t odd_index  = even_index + 1;
		decoded_data[ii] = (candidate_decoded_messages[odd_index][0] << 4) +
				candidate_decoded_messages[even_index][0];
	}

}

uint32_t WickedReceiver::getMillisSinceLastDecode(void){
	return (millis() - timestamp_of_last_decode);
}

uint8_t WickedReceiver::inSilenceWindow(void){
	return (getMillisSinceLastDecode() > STATISTICS_SILENT_WINDOW_MS) ? 1 : 0;
}

uint8_t WickedReceiver::getDecodedNodeId(void){
	return getDecodedByte(0);
}

uint16_t WickedReceiver::getDecodedPacketNumber(void){
	return getDecodedByte(1) + getDecodedByte(2)*256;
}

uint8_t WickedReceiver::getDecodedSensor1(void){
  return getDecodedByte(3);
}

uint8_t WickedReceiver::getDecodedSensor2(void){
  return getDecodedByte(4);
}

uint8_t WickedReceiver::getDecodedSensor3(void){
  return getDecodedByte(5);
}

uint8_t WickedReceiver::getDecodedDigitalCount(void){
  return getDecodedByte(6);
}

uint8_t  WickedReceiver::getDecodedByte(uint8_t byte_number){
  if(byte_number < NUM_BYTES_IN_DECODED_PACKET){
    return decoded_data[byte_number];
  }
  return -1;
}

uint8_t  WickedReceiver::getEncodedByte(uint8_t byte_number){
  if(byte_number < NUM_BYTES_IN_ENCODED_PACKET){
    return inbyte[byte_number];
  }
  return -1;  
}

void WickedReceiver::nextPacket(void){
	packet_ready = 0;
}
