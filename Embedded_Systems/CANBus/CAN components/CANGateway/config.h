//  can 1 -> tx: 22, rx: 23 -> line with MPPTs
//  can 2 -> tx: 1, rx: 0 -> line with VESCs
//  can 3 -> tx 31, rx: 30 -> line with Walter

// ---------------------------------------------------------------------------
// ID's which mean that safe state should be initiated
// ---------------------------------------------------------------------------
const uint32_t EMERGENCY_CANIDS[] = {

};

// ---------------------------------------------------------------------------
//  Forwarding tables — STANDARD frames (11-bit IDs, 0x000 to 0x7FF)
// ---------------------------------------------------------------------------

// ---- CAN1 → CAN2 (standard) -----------------------------------------------
const uint32_t STD_CAN1_TO_CAN2[] = {
  0x74, // Lights
  0x75 // Lights
};

// ---- CAN1 → CAN3 (standard) -----------------------------------------------
const uint32_t STD_CAN1_TO_CAN3[] = {
  0x10, // Steering angle sensor error
  0x11, // Steering angle sensor
  0x20, // Accelerator error
  0x21, // Accelerator
  0x200, // MPPT 0 power
  0x216, // MPPT 1 power
  0x232, // MPPT 2 power
  0x248, // MPPT 3 power
  0x201, // MPPT 0 status
  0x217, // MPPT 1 status
  0x233, // MPPT 2 status
  0x249, // MPPT 3 status
  0x74, // Lights
  0x75 // Lights
};

// ---- CAN2 → CAN1 (standard) -----------------------------------------------
const uint32_t STD_CAN2_TO_CAN1[] = {
  0x74, // Lights
  0x75 // Lights
};

// ---- CAN2 → CAN3 (standard) -----------------------------------------------
const uint32_t STD_CAN2_TO_CAN3[] = {
  0x001, // BMS heartbeat
  0x100, // BMS status
  0x150, // PDU error
  0x151, // PDU current draw
  0x74, // Lights
  0x75 // Lights
};

// ---- CAN3 → CAN1 (standard) -----------------------------------------------
const uint32_t STD_CAN3_TO_CAN1[] = {
  0x74, // Lights
  0x75 // Lights
};

// ---- CAN3 → CAN2 (standard) -----------------------------------------------
const uint32_t STD_CAN3_TO_CAN2[] = {
  0x74, // Lights
  0x75 // Lights
};

// ---------------------------------------------------------------------------
//  Forwarding tables — EXTENDED frames (29-bit IDs, up to 0x1FFFFFFF)
// ---------------------------------------------------------------------------

// ---- CAN1 → CAN2 (extended) -----------------------------------------------
const uint32_t EXT_CAN1_TO_CAN2[] = {

};

// ---- CAN1 → CAN3 (extended) -----------------------------------------------
const uint32_t EXT_CAN1_TO_CAN3[] = {

};

// ---- CAN2 → CAN1 (extended) -----------------------------------------------
const uint32_t EXT_CAN2_TO_CAN1[] = {

};

// ---- CAN2 → CAN3 (extended) -----------------------------------------------
const uint32_t EXT_CAN2_TO_CAN3[] = {
  0x00000901, // VESC 1 status 1 
  0x00000902, // VESC 2 status 1
  0x00000E01, // VESC 1 status 2 
  0x00000E02, // VESC 2 status 2
  0x00000F01, // VESC 1 status 3 
  0x00000F02, // VESC 2 status 3
  0x00001001, // VESC 1 status 4 
  0x00001002, // VESC 2 status 4
  0x00001B01, // VESC 1 status 5 
  0x00001B02, // VESC 2 status 5
};

// ---- CAN3 → CAN1 (extended) -----------------------------------------------
const uint32_t EXT_CAN3_TO_CAN1[] = {

}; 

// ---- CAN3 → CAN2 (extended) -----------------------------------------------
const uint32_t EXT_CAN3_TO_CAN2[] = {

};