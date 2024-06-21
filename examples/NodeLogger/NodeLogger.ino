#define RADIO_BAUD_RATE   2400
#define UART_BAUD_RATE  115200

#include <WickedReceiver.h>

#define FILTER_NODE_ID     15

WickedReceiver receiver;
int rx_status = 0;

int numBytesAvailable = 0;
int incomingByte = 0;

long now = 0;
byte flag = -1;

void setup(){
  Serial.begin(UART_BAUD_RATE);
  Serial.println("Hello World");
  
  Serial.flush();
  delay(1000);
  Serial.begin(RADIO_BAUD_RATE);
}  


void loop(){ 
  numBytesAvailable = Serial.available();  
  for(int ii=0; ii < numBytesAvailable; ii++){   
    incomingByte = Serial.read();    
    rx_status = receiver.processEncodedByte(incomingByte);
    if(rx_status == 1){
      printRadioPacket();

      // do other stuff with the packet

      receiver.nextPacket();
    }
  }
  
}

void printRadioPacket(){
  Serial.flush();
  delay(1000);
  Serial.begin(UART_BAUD_RATE); 
  //print using random access
  
  Serial.println();
  
  Serial.print(millis(), DEC);
  Serial.print("\t\t");
  for(int i = 0; i < 8; i++){
    Serial.print(receiver.getDecodedByte(i), DEC);
    Serial.print("\t");
  } 
  Serial.println();
  Serial.print("\t");  
  Serial.print("\t");
  Serial.print(receiver.getDecodedNodeId(), DEC);
  Serial.print("\t");  
  Serial.print(receiver.getDecodedPacketNumber(), DEC);
  Serial.print("\t");  
  Serial.print("\t");  
  Serial.print(receiver.getDecodedSensor1(), DEC);
  Serial.print("\t");  
  Serial.print(receiver.getDecodedSensor2(), DEC);
  Serial.print("\t");  
  Serial.print(receiver.getDecodedSensor3(), DEC);
  Serial.print("\t");  
  Serial.print(receiver.getDecodedDigitalCount(), DEC);
  Serial.println();

  Serial.flush();
  delay(1000);  
  Serial.begin(RADIO_BAUD_RATE); 
}