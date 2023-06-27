/*
  #############################################################################################
  # This file is part of U_BMS_ESP32_CAN <https://github.com/cheezemanrich/U_BMS_ESP32_CAN>.
  #
  # U_BMS_ESP32_CAN is free software: you can redistribute it and/or modify
  # it under the terms of the GNU General Public License as published by
  # the Free Software Foundation, either version 3 of the License, or
  # (at your option) any later version.
  #
  # BMScan is distributed in the hope that it will be useful,
  # but WITHOUT ANY WARRANTY; without even the implied warranty of
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  # GNU General Public License for more details.
  #############################################################################################
	
	This code is based on:
	Copyright (C) designer2k2 Stephan M.
  # This file is part of EMUcan <https://github.com/designer2k2/EMUcan>.
  #
  # EMUcanT4 is free software: you can redistribute it and/or modify
  # it under the terms of the GNU General Public License as published by
  # the Free Software Foundation, either version 3 of the License, or
  # (at your option) any later version.
  #
  # EMUcan is distributed in the hope that it will be useful,
  # but WITHOUT ANY WARRANTY; without even the implied warranty of
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  # GNU General Public License for more details.
  #
  # You should have received a copy of the GNU General Public License
  # along with EMUcan.  If not, see <http://www.gnu.org/licenses/>.
  
*/

#include "U_BMS_ESP32_CAN.h"


BMScan::BMScan(const uint32_t BMSbase) {
  //Getting the base number, as set in the EMU Software
  _BMSbase = BMSbase;
}

bool BMScan::checkBMScan(uint32_t can_id, uint8_t can_dlc, uint8_t data[8]) {
  //Check if Message is within Range of of BMS:
  if (can_id >= _BMSbase && can_id <= _BMSbase + 0xFFE) {   // Range = Base (0x001) to 0xFFE
    //So messages here should be decoded!
    _decodeBMSFrame(can_id, can_dlc, data);
    //Store the event:
    _bmscanstatusEngine(BMS_MESSAGE_RECEIVED_VALID);
    return true;
  } else {
    _bmscanstatusEngine(BMS_RECEIVED_NOTHING);
    return false;
  }
}

void BMScan::_bmscanstatusEngine(const BMS_STATUS_UPDATES action) {
  //check the current time versus the last to define the status.
  unsigned long currentMillis = millis();
  switch (action) {
    case BMS_RECEIVED_NOTHING:
      if (currentMillis - _previousMillis >= 1000) {
        _BMScan_Status = BMScan_RECEIVED_NOTHING_WITHIN_LAST_SECOND;
      }
      break;
    case BMS_MESSAGE_RECEIVED_VALID:
      _previousMillis = currentMillis;
      _BMScan_Status = BMScan_RECEIVED_WITHIN_LAST_SECOND;
      break;
    default:
      break;
  }
}

BMScan_STATUS BMScan::BMScan_Status() {
  _bmscanstatusEngine(BMS_RECEIVED_NOTHING);
  return _BMScan_Status;
}

void BMScan::_decodeBMSFrame(uint32_t can_id, uint8_t can_dlc, uint8_t data[8]) {
  //This decodes the frames and fills them into the data:

  if (can_id == 0x0C0) {   // 0x0C1 - U-BMS_VMU_STATUS
    bms_data.bat_soc = data[0];
    bms_data.bat_status1 = data[1];
    bms_data.bat_status2 = data[2];
    bms_data.bat_status3 = data[3];
    bms_data.bat_status4 = data[4];
    bms_data.bat_num_mods = data[5];
    bms_data.bat_num_bal = data[6];
    bms_data.bat_status5 = data[7];
  }
  if (can_id == 0x0C1) {   // 0x0C1 - U-BMS_VMU_INFO
    // Empty
  }
  if (can_id == 0x0C2) {   // 0x0C2 - U-BMS_VMU_CHARGE
    // Empty
  }
  if (can_id == 0x0C4) {   // 0x0C4 - U-BMS_VMU_TRACE
    bms_data.bat_max_temp = (map(data[0], 0, 255, -40, 215)); // 0=-40'C, 255=215'C
    bms_data.bat_min_temp = (map(data[1], 0, 255, -40, 215)); // 0=-40'C, 255=215'C
    bms_data.bat_pcb_max_temp = (map(data[0], 0, 255, -40, 215)); // 0=-40'C, 255=215'C
    bms_data.cell_max_v = ((data[5] << 8) + data[4]); // mV
    bms_data.cell_min_v = ((data[7] << 8) + data[6]); // mV
  }
  if (can_id == 0x180) {   // 0x180 - Hardware & firmware revision
    bms_data.fw_code_rev = data[0];
    bms_data.cust_rel_rev = data[1];
    bms_data.boot_load_rev = data[2];
    bms_data.bms_type = data[3];
    bms_data.hw_rev = data[4];
    bms_data.cust_code_1 = data[5];
    bms_data.cust_code_2 = data[6];
    bms_data.cust_code_3 = data[7];
  }
  if (can_id == 0x350) {   // 0x350 - Module 1 cell voltages 1-3 or 7-9
    bms_data.mod1_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod1_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod1_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x351) {    // 0x351 - Module 1 cell voltages 4-6 or 10-12
    bms_data.mod1_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x352) {   // 0x352 - Module 2 cell voltages 1-3 or 7-9
    bms_data.mod2_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod2_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod2_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x353) {    // 0x353 - Module 2 cell voltages 4-6 or 10-12
    bms_data.mod2_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x354) {   // 0x354 - Module 3 cell voltages 1-3 or 7-9
    bms_data.mod3_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod3_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod3_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x355) {    // 0x355 - Module 3 cell voltages 4-6 or 10-12
    bms_data.mod3_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x356) {   // 0x356 - Module 4 cell voltages 1-3 or 7-9
    bms_data.mod4_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod4_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod4_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x357) {    // 0x357 - Module 4 cell voltages 4-6 or 10-12
    bms_data.mod4_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x358) {   // 0x358 - Module 5 cell voltages 1-3 or 7-9
    bms_data.mod5_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod5_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod5_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x359) {    // 0x359 - Module 5 cell voltages 4-6 or 10-12
    bms_data.mod5_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x35A) {   // 0x36A - Module 6 cell voltages 1-3 or 7-9
    bms_data.mod6_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod6_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod6_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x35B) {    // 0x36B - Module 6 cell voltages 4-6 or 10-12
    bms_data.mod6_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x35C) {   // 0x36C - Module 7 cell voltages 1-3 or 7-9
    bms_data.mod7_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod7_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod7_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x35D) {    // 0x36D - Module 7 cell voltages 4-6 or 10-12
    bms_data.mod7_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x35E) {   // 0x36E - Module 8 cell voltages 1-3 or 7-9
    bms_data.mod8_cell_1_V = (data[2] << 8) + data[3];
    bms_data.mod8_cell_2_V = (data[4] << 8) + data[5];
    bms_data.mod8_cell_3_V = (data[6] << 8) + data[7];
  }
  if (can_id == 0x35F) {    // 0x36F - Module 8 cell voltages 4-6 or 10-12
    bms_data.mod8_cell_4_V = (data[2] << 8) + data[3];
  }
  if (can_id == 0x46A) {    // 0x46A - Module 1-3  current
    float mod1_curr_temp = (data[2] << 8) + data[3];
    float mod2_curr_temp = (data[4] << 8) + data[5];
    float mod3_curr_temp = (data[6] << 8) + data[7];
    if (mod1_curr_temp > 32767){ // convert to a negative value
      mod1_curr_temp = mod1_curr_temp - 65535;
    }
    if (mod2_curr_temp > 32767){ // convert to a negative value
      mod2_curr_temp = mod2_curr_temp - 65535;
    }
    if (mod3_curr_temp > 32767){ // convert to a negative value
      mod3_curr_temp = mod3_curr_temp - 65535;
    }
    bms_data.mod1_curr = mod1_curr_temp / 100;
    bms_data.mod2_curr = mod2_curr_temp / 100;
    bms_data.mod3_curr = mod3_curr_temp / 100;
  }
  if (can_id == 0x46B) {    // 0x46B - Module 4-6 current
    float mod4_curr_temp = (data[2] << 8) + data[3];
    float mod5_curr_temp = (data[4] << 8) + data[5];
    float mod6_curr_temp = (data[6] << 8) + data[7];
    if (mod4_curr_temp > 32767){ // convert to a negative value
      mod4_curr_temp = mod4_curr_temp - 65535;
    }
    if (mod5_curr_temp > 32767){ // convert to a negative value
      mod5_curr_temp = mod5_curr_temp - 65535;
    }
    if (mod6_curr_temp > 32767){ // convert to a negative value
      mod6_curr_temp = mod6_curr_temp - 65535;
    }
    bms_data.mod4_curr = mod4_curr_temp / 100;
    bms_data.mod5_curr = mod5_curr_temp / 100;
    bms_data.mod6_curr = mod6_curr_temp / 100;
  }
  if (can_id == 0x46C) {    // 0x46C - Module 7-9 current
    float mod7_curr_temp = (data[2] << 8) + data[3];
    float mod8_curr_temp = (data[4] << 8) + data[5];
    if (mod7_curr_temp > 32767){ // convert to a negative value
      mod7_curr_temp = mod7_curr_temp - 65535;
    }
    if (mod8_curr_temp > 32767){ // convert to a negative value
      mod8_curr_temp = mod8_curr_temp - 65535;
    }
    bms_data.mod7_curr = mod7_curr_temp / 100;
    bms_data.mod8_curr = mod8_curr_temp / 100;
  }
  if (can_id == 0x56A) {    // 0x56A - Module 1-8 exists flags
    bms_data.mod_exists = data[1];  // module 1-8, 0=Absent, 1=Exists
  }
  if (can_id == 0x66A) {    // 0x66A - Insulation Resistance
    // Empty
  }
  if (can_id == 0x66B) {    // 0x66B - Insulation Voltage
    // Empty
  }
  if (can_id == 0x76A) {    // 0x76A - Module 1-3 Temperature
    uint16_t mod1_temp_temp = (data[2] * 256) + data[3];
    uint16_t mod2_temp_temp = (data[4] * 256) + data[5];
    uint16_t mod3_temp_temp = (data[6] * 256) + data[7];
    bms_data.mod1_temp = (float)mod1_temp_temp / 100;
    bms_data.mod2_temp = (float)mod2_temp_temp / 100;
    bms_data.mod3_temp = (float)mod3_temp_temp / 100;
  }
  if (can_id == 0x76B) {    // 0x76B - Module 4-6 Temperature
    uint16_t mod4_temp_temp = (data[2] * 256) + data[3];
    uint16_t mod5_temp_temp = (data[4] * 256) + data[5];
    uint16_t mod6_temp_temp = (data[6] * 256) + data[7];
    bms_data.mod4_temp = (float)mod4_temp_temp / 100;
    bms_data.mod5_temp = (float)mod5_temp_temp / 100;
    bms_data.mod6_temp = (float)mod6_temp_temp / 100;
  }
  if (can_id == 0x76C) {    // 0x76C - Module 7-9 Temperature
    uint16_t mod7_temp_temp = (data[2] * 256) + data[3];
    uint16_t mod8_temp_temp = (data[4] * 256) + data[5];
    bms_data.mod7_temp = (float)mod7_temp_temp / 100;
    bms_data.mod8_temp = (float)mod8_temp_temp / 100;
  }
  if (can_id == 0x06A) {    // 0x06A - Module 1-7 SOC
    bms_data.mod_1_soc = (map(data[1], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
    bms_data.mod_2_soc = (map(data[2], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
    bms_data.mod_3_soc = (map(data[3], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
    bms_data.mod_4_soc = (map(data[4], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
    bms_data.mod_5_soc = (map(data[5], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
    bms_data.mod_6_soc = (map(data[6], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
    bms_data.mod_7_soc = (map(data[7], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
  }
  if (can_id == 0x06B) {    // 0x06B - Module 8-10 SOC
    bms_data.mod_8_soc = (map(data[1], 0, 255, 0, 100)); // convert 0-255 value to 0-100% value
  }
  if (can_id == 0x16A) {    // 0x16A - Module 1-55 inter balance flags, each byte is 8 modules
    bms_data.mod_bal_stat = data[1];  // module 1-8, 0=Active, 1=Inactive
  }
  if (can_id == 0x16C) {    // 0x16C - Module 1-55 Sanity Error flags
    bms_data.mod_sanity_error_flags = data[1];  // module 1-8, 0=No Error, 1=Error
  }
  if (can_id == 0x26A) {    // 0x26A - Module 1-7 Intra-balance flags, cells 1-8
    bms_data.mod1_cell_bal_stat = data[1];  // module 1, cell 1-8, 0=Active, 1=Inactive
    bms_data.mod2_cell_bal_stat = data[2];  // module 2, cell 1-8, 0=Active, 1=Inactive
    bms_data.mod3_cell_bal_stat = data[3];  // module 3, cell 1-8, 0=Active, 1=Inactive
    bms_data.mod4_cell_bal_stat = data[4];  // module 4, cell 1-8, 0=Active, 1=Inactive
    bms_data.mod5_cell_bal_stat = data[5];  // module 5, cell 1-8, 0=Active, 1=Inactive
    bms_data.mod6_cell_bal_stat = data[6];  // module 6, cell 1-8, 0=Active, 1=Inactive
    bms_data.mod7_cell_bal_stat = data[7];  // module 7, cell 1-8, 0=Active, 1=Inactive
  }
  if (can_id == 0x26B) {    // 0x26B - Module 8-10 Intra-balance flags, cells 1-8
    bms_data.mod8_cell_bal_stat = data[1];  // module 8, cell 1-8, 0=Active, 1=Inactive
  }
  if (can_id == 0x67A) {    // 0x67A - Module 1-3 PCBA temperature
    uint16_t mod1_pcb_temp_temp = (data[2] * 256) + data[3];
    uint16_t mod2_pcb_temp_temp = (data[4] * 256) + data[5];
    uint16_t mod3_pcb_temp_temp = (data[6] * 256) + data[7];
    bms_data.mod1_pcb_temp = (float)mod1_pcb_temp_temp / 100;
    bms_data.mod2_pcb_temp = (float)mod2_pcb_temp_temp / 100;
    bms_data.mod3_pcb_temp = (float)mod3_pcb_temp_temp / 100;
  }
  if (can_id == 0x67B) {    // 0x67B - Module 4-7 PCBA temperature
    uint16_t mod4_pcb_temp_temp = (data[2] * 256) + data[3];
    uint16_t mod5_pcb_temp_temp = (data[4] * 256) + data[5];
    uint16_t mod6_pcb_temp_temp = (data[6] * 256) + data[7];
    bms_data.mod4_pcb_temp = (float)mod4_pcb_temp_temp / 100;
    bms_data.mod5_pcb_temp = (float)mod5_pcb_temp_temp / 100;
    bms_data.mod6_pcb_temp = (float)mod6_pcb_temp_temp / 100;
  }
  if (can_id == 0x67C) {    // 0x67C - Module 8-10 PCBA temperature
    uint16_t mod7_pcb_temp_temp = (data[2] * 256) + data[3];
    uint16_t mod8_pcb_temp_temp = (data[4] * 256) + data[5];
    bms_data.mod7_pcb_temp = (float)mod7_pcb_temp_temp / 100;
    bms_data.mod8_pcb_temp = (float)mod8_pcb_temp_temp / 100;
  }
  if (can_id == 0x184) {    // 0x184 - BMS, Module TLA# and Firmware Revision
    // Empty
  }
}

