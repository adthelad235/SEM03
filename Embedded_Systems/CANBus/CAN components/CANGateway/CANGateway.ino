#include <FlexCAN_T4.h>
#include "config.h"

//  Bus objects
//  can 1 -> tx: 22, rx: 23
//  can 2 -> tx: 1, rx: 0
//  can 3 -> tx 31, rx: 30
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

//  Check whether an ID is present in a forwarding table
template<size_t N> bool idInTable(uint32_t id, const uint32_t (&table)[N]) {
  for (size_t i = 0; i < N; i++) {
    if (table[i] == id) return true;
  }
  return false;
}

//  Forward a frame to a destination bus, preserving the extended flag
void forwardFrame(FlexCAN_T4_Base *dest, const CAN_message_t &msg) {
  CAN_message_t out = msg;  // copy frame including id, len, buf, flags
  if      (dest == (FlexCAN_T4_Base*)&can1) can1.write(out);
  else if (dest == (FlexCAN_T4_Base*)&can2) can2.write(out);
  else if (dest == (FlexCAN_T4_Base*)&can3) can3.write(out);
}

//  Print a forwarded frame to Serial for debugging
void printFrame(const CAN_message_t &msg, const char *label) {
  Serial.print(label);
  Serial.print(msg.flags.extended ? " [EXT]" : " [STD]");
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data:");
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(" ");
    if (msg.buf[i] < 0x10) Serial.print("0");
    Serial.print(msg.buf[i], HEX);
  }
  Serial.println();
}

//  Check a message against one forwarding table and forward if matched.
//  isExtended selects which frame type this table applies to.
template<size_t N> void routeIfMatch(const CAN_message_t &msg, bool isExtended, const uint32_t (&table)[N], FlexCAN_T4_Base *dest, const char *label) {
  // Only process frames of the correct type for this table
  if ((bool)msg.flags.extended != isExtended) return;

  if (N == 0) return;

  if (idInTable(msg.id, table)) {
    forwardFrame(dest, msg);
    printFrame(msg, label);
  }
}

//  Process all routing rules for one received message
void routeMessage(const CAN_message_t &msg, uint8_t sourceBus) {
  //checkSafeState(msg);
  switch (sourceBus) {

    case 1:
      // Standard frames from CAN1
      routeIfMatch(msg, false, STD_CAN1_TO_CAN2, (FlexCAN_T4_Base*)&can2, "[CAN1->CAN2]");
      routeIfMatch(msg, false, STD_CAN1_TO_CAN3, (FlexCAN_T4_Base*)&can3, "[CAN1->CAN3]");
      // Extended frames from CAN1
      //routeIfMatch(msg, true,  EXT_CAN1_TO_CAN2, (FlexCAN_T4_Base*)&can2, "[CAN1->CAN2]");
      //routeIfMatch(msg, true,  EXT_CAN1_TO_CAN3, (FlexCAN_T4_Base*)&can3, "[CAN1->CAN3]");
      break;

    case 2:
      // Standard frames from CAN2
      routeIfMatch(msg, false, STD_CAN2_TO_CAN1, (FlexCAN_T4_Base*)&can1, "[CAN2->CAN1]");
      routeIfMatch(msg, false, STD_CAN2_TO_CAN3, (FlexCAN_T4_Base*)&can3, "[CAN2->CAN3]");
      // Extended frames from CAN2
      //routeIfMatch(msg, true,  EXT_CAN2_TO_CAN1, (FlexCAN_T4_Base*)&can1, "[CAN2->CAN1]");
      routeIfMatch(msg, true,  EXT_CAN2_TO_CAN3, (FlexCAN_T4_Base*)&can3, "[CAN2->CAN3]");
      break;

    case 3:
      // Standard frames from CAN3
      routeIfMatch(msg, false, STD_CAN3_TO_CAN1, (FlexCAN_T4_Base*)&can1, "[CAN3->CAN1]");
      routeIfMatch(msg, false, STD_CAN3_TO_CAN2, (FlexCAN_T4_Base*)&can2, "[CAN3->CAN2]");
      // Extended frames from CAN3
      //routeIfMatch(msg, true,  EXT_CAN3_TO_CAN1, (FlexCAN_T4_Base*)&can1, "[CAN3->CAN1]");
      //routeIfMatch(msg, true,  EXT_CAN3_TO_CAN2, (FlexCAN_T4_Base*)&can2, "[CAN3->CAN2]");
      break;
  }
}

// bool checkSafeState(const CAN_message_t &msg){
//   if (idInTable(msg.id, EMERGENCY_CANIDS)){
//     sendSafeState();
//   }
// }

void sendSafeState(){
  // Prepare CAN message
  CAN_message_t msg;
  msg.id = 0x01;    // CAN ID
  msg.len = 0;  // Data length

  // Transmit message
  can1.write(msg);
  can2.write(msg);
  can3.write(msg);
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("CAN Gateway starting...");

  can1.begin();
  can1.setBaudRate(500000);

  can2.begin();
  can2.setBaudRate(500000);

  can3.begin();
  can3.setBaudRate(500000);

  Serial.println("CAN Gateway ready.");
}

void loop() {
  CAN_message_t msg;

  while (can1.read(msg)) routeMessage(msg, 1);
  while (can2.read(msg)) routeMessage(msg, 2);
  while (can3.read(msg)) routeMessage(msg, 3);
}
