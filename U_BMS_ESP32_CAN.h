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

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef _U_BM_ESP32_CAN_h
#define _U_BM_ESP32_CAN_h

#define BMSCAN_LIB_VERSION (F("1.0.0"))

// Available data
struct bms_data_t {
  uint8_t bat_max_temp;
  uint8_t bat_min_temp;
  uint8_t bat_pcb_max_temp;
  uint16_t cell_max_v;
  uint16_t cell_min_v;
  uint8_t fw_code_rev;
  uint8_t cust_rel_rev;
  uint8_t boot_load_rev;
  uint8_t bms_type;
  uint8_t hw_rev;
  uint8_t cust_code_1;
  uint8_t cust_code_2;
  uint8_t cust_code_3;
  uint8_t bat_soc;
  uint8_t bat_status1;
  uint8_t bat_status2;
  uint8_t bat_status3;
  uint8_t bat_status4;
  uint8_t bat_status5;
  uint8_t bat_num_mods;
  uint8_t bat_num_bal;
  uint8_t mod_1_soc;
  uint8_t mod_2_soc;
  uint8_t mod_3_soc;
  uint8_t mod_4_soc;
  uint8_t mod_5_soc;
  uint8_t mod_6_soc;
  uint8_t mod_7_soc;
  uint8_t mod_8_soc;
  uint8_t mod_exists;
  uint8_t mod_bal_stat;
  uint8_t mod_sanity_error_flags;
  uint8_t mod1_cell_bal_stat;
  uint8_t mod2_cell_bal_stat;
  uint8_t mod3_cell_bal_stat;
  uint8_t mod4_cell_bal_stat;
  uint8_t mod5_cell_bal_stat;
  uint8_t mod6_cell_bal_stat;
  uint8_t mod7_cell_bal_stat;
  uint8_t mod8_cell_bal_stat;
  float mod1_curr;
  float mod2_curr;
  float mod3_curr;
  float mod4_curr;
  float mod5_curr;
  float mod6_curr;
  float mod7_curr;
  float mod8_curr;
  float mod1_temp;
  float mod2_temp;
  float mod3_temp;
  float mod4_temp;
  float mod5_temp;
  float mod6_temp;
  float mod7_temp;
  float mod8_temp;
  float mod1_pcb_temp;
  float mod2_pcb_temp;
  float mod3_pcb_temp;
  float mod4_pcb_temp;
  float mod5_pcb_temp;
  float mod6_pcb_temp;
  float mod7_pcb_temp;
  float mod8_pcb_temp;
  uint16_t mod1_cell_1_V;
  uint16_t mod1_cell_2_V;
  uint16_t mod1_cell_3_V;
  uint16_t mod1_cell_4_V;
  uint16_t mod2_cell_1_V;
  uint16_t mod2_cell_2_V;
  uint16_t mod2_cell_3_V;
  uint16_t mod2_cell_4_V;
  uint16_t mod3_cell_1_V;
  uint16_t mod3_cell_2_V;
  uint16_t mod3_cell_3_V;
  uint16_t mod3_cell_4_V;
  uint16_t mod4_cell_1_V;
  uint16_t mod4_cell_2_V;
  uint16_t mod4_cell_3_V;
  uint16_t mod4_cell_4_V;
  uint16_t mod5_cell_1_V;
  uint16_t mod5_cell_2_V;
  uint16_t mod5_cell_3_V;
  uint16_t mod5_cell_4_V;
  uint16_t mod6_cell_1_V;
  uint16_t mod6_cell_2_V;
  uint16_t mod6_cell_3_V;
  uint16_t mod6_cell_4_V;
  uint16_t mod7_cell_1_V;
  uint16_t mod7_cell_2_V;
  uint16_t mod7_cell_3_V;
  uint16_t mod7_cell_4_V;
  uint16_t mod8_cell_1_V;
  uint16_t mod8_cell_2_V;
  uint16_t mod8_cell_3_V;
  uint16_t mod8_cell_4_V;
};

enum BMScan_STATUS {
  BMScan_FRESH,
  BMScan_RECEIVED_WITHIN_LAST_SECOND,
  BMScan_RECEIVED_NOTHING_WITHIN_LAST_SECOND
};

class BMScan {
  
public:
  // Constructor
  BMScan(const uint32_t BMSbase = 0x0C0);

  // Methods
  bool checkBMScan(uint32_t can_id, uint8_t can_dlc, uint8_t data[8]);
//  bool decodeCel();
  BMScan_STATUS BMScan_Status();

  // Data
  struct bms_data_t bms_data;

  // Privates
private:

  enum BMScan_STATUS _BMScan_Status = BMScan_FRESH;

  enum BMS_STATUS_UPDATES {
    BMS_MESSAGE_RECEIVED_VALID,
    BMS_RECEIVED_NOTHING
  };

  void _decodeBMSFrame(uint32_t can_id, uint8_t can_dlc, uint8_t data[8]);
  void _bmscanstatusEngine(const BMS_STATUS_UPDATES action);

  uint32_t _BMSbase;
  unsigned long _previousMillis = 0;
};
#endif
