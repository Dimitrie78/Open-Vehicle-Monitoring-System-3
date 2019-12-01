/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          1th October 2018
 ;
 ;    Changes:
 ;    1.0  Initial release
 ;
 ;    (C) 2018       Martin Graml
 ;    (C) 2019       Thomas Heuer
 ;
 ; Permission is hereby granted, free of charge, to any person obtaining a copy
 ; of this software and associated documentation files (the "Software"), to deal
 ; in the Software without restriction, including without limitation the rights
 ; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 ; copies of the Software, and to permit persons to whom the Software is
 ; furnished to do so, subject to the following conditions:
 ;
 ; The above copyright notice and this permission notice shall be included in
 ; all copies or substantial portions of the Software.
 ;
 ; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 ; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 ; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 ; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 ; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 ; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 ; THE SOFTWARE.
 ;
 ; Most of the CAN Messages are based on https://github.com/MyLab-odyssey/ED_BMSdiag
 ; http://ed.no-limit.de/wiki/index.php/Hauptseite
 
const PROGMEM byte rqBattHWrev[4]                 = {0x03, 0x22, 0xF1, 0x50};
const PROGMEM byte rqBattSWrev[4]                 = {0x03, 0x22, 0xF1, 0x51};
const PROGMEM byte rqBattVIN[4]                   = {0x03, 0x22, 0xF1, 0x90};
const PROGMEM byte rqBattTemperatures[4]          = {0x03, 0x22, 0x02, 0x01}; 
const PROGMEM byte rqBattModuleTemperatures[4]    = {0x03, 0x22, 0x02, 0x02};
const PROGMEM byte rqBattHVstatus[4]              = {0x03, 0x22, 0x02, 0x04};
const PROGMEM byte rqBattADCref[4]                = {0x03, 0x22, 0x02, 0x07};
const PROGMEM byte rqBattVolts[6]                 = {0x03, 0x22, 0x02, 0x08, 28, 57};
const PROGMEM byte rqBattIsolation[4]             = {0x03, 0x22, 0x02, 0x09};
const PROGMEM byte rqBattAmps[4]                  = {0x03, 0x22, 0x02, 0x03};
const PROGMEM byte rqBattDate[4]                  = {0x03, 0x22, 0x03, 0x04};
const PROGMEM byte rqBattProdDate[4]              = {0x03, 0x22, 0xF1, 0x8C};
const PROGMEM byte rqBattCapacity[6]              = {0x03, 0x22, 0x03, 0x10, 31, 59};
const PROGMEM byte rqBattHVContactorCyclesLeft[4] = {0x03, 0x22, 0x03, 0x0B};
const PROGMEM byte rqBattHVContactorMax[4]        = {0x03, 0x22, 0x03, 0x0C};
const PROGMEM byte rqBattHVContactorState[4]      = {0x03, 0x22, 0xD0, 0x00};

const PROGMEM byte rqCarVIN[4]                   = {0x02, 0x09, 0x02, 0x00};

//Experimental readouts
const PROGMEM byte rqBattCapInit[4]               = {0x03, 0x22, 0x03, 0x05};
const PROGMEM byte rqBattCapLoss[4]               = {0x03, 0x22, 0x03, 0x09};
const PROGMEM byte rqBattUnknownCounter[4]        = {0x03, 0x22, 0x01, 0x01};

// NLG6-Charger module
const PROGMEM byte rqChargerPN_HW[4]              = {0x03, 0x22, 0xF1, 0x11};
const PROGMEM byte rqChargerSWrev[4]              = {0x03, 0x22, 0xF1, 0x21};
const PROGMEM byte rqChargerVoltages[4]           = {0x03, 0x22, 0x02, 0x26};
const PROGMEM byte rqChargerAmps[4]               = {0x03, 0x22, 0x02, 0x25};
const PROGMEM byte rqChargerSelCurrent[4]         = {0x03, 0x22, 0x02, 0x2A};
const PROGMEM byte rqChargerTemperatures[4]       = {0x03, 0x22, 0x02, 0x23};
 */

#include "ovms_log.h"
static const char *TAG = "v-smarted";

#include <stdio.h>
#include <algorithm>
#include <string>
#include <iomanip>
#include "pcp.h"
#include "ovms_metrics.h"
#include "ovms_events.h"
#include "ovms_config.h"
#include "ovms_command.h"
#include "metrics_standard.h"
#include "ovms_notify.h"
#include "ovms_utils.h"

#include "vehicle_smarted.h"

#undef SQR
#define SQR(n) ((n)*(n))
#undef ABS
#define ABS(n) (((n) < 0) ? -(n) : (n))
#undef LIMIT_MIN
#define LIMIT_MIN(n,lim) ((n) < (lim) ? (lim) : (n))

#undef ROUNDPREC
#define ROUNDPREC(fval,prec) (round((fval) * pow(10,(prec))) / pow(10,(prec)))

static const OvmsVehicle::poll_pid_t smarted_polls[] =
{
  { 0x61A, 0x483, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xF111, {  0,120,999,0 } }, // rqChargerPN_HW
  { 0x61A, 0x483, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0226, {  0,120,999,0 } }, // rqChargerVoltages
  { 0x61A, 0x483, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0225, {  0,120,999,0 } }, // rqChargerAmps
  { 0x61A, 0x483, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x022A, {  0,120,999,0 } }, // rqChargerSelCurrent
  { 0x61A, 0x483, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0223, {  0,120,999,0 } }, // rqChargerTemperatures
  { 0x7E7, 0x7EF, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0201, {  0,300,600,0 } }, // rqBattTemperatures
  { 0x7E7, 0x7EF, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0202, {  0,300,600,0 } }, // rqBattModuleTemperatures
  { 0x7E7, 0x7EF, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0208, {  0,60,600,0 } }, // rqBattVolts
  { 0x7E7, 0x7EF, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0209, {  0,60,600,0 } }, // rqBattIsolation
  { 0x7E7, 0x7EF, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0203, {  0,60,600,0 } }, // rqBattAmps
  { 0x7E7, 0x7EF, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x0310, {  0,60,600,0 } }, // rqBattCapacity
  { 0, 0, 0x00, 0x00, { 0, 0, 0, 0 } }
};

void OvmsVehicleSmartED::ObdInitPoll() {
  
  m_bms_capacitys = NULL;
  m_bms_cmins = NULL;
  m_bms_cmaxs = NULL;
  m_bms_cdevmaxs = NULL;
  m_bms_calerts = NULL;
  m_bms_calerts_new = 0;
  m_bms_has_capacitys = false;

  m_bms_bitset_c.clear();
  m_bms_bitset_cc = 0;
  m_bms_readings_c = 0;
  m_bms_readingspermodule_c = 0;

  m_bms_limit_cmin = 1000;
  m_bms_limit_cmax = 22000;

  mt_v_bat_pack_cmin = new OvmsMetricFloat("xse.v.b.p.capacity.min", SM_STALE_HIGH, Other);
  mt_v_bat_pack_cmax = new OvmsMetricFloat("xse.v.b.p.capacity.max", SM_STALE_HIGH, Other);
  mt_v_bat_pack_cavg = new OvmsMetricFloat("xse.v.b.p.capacity.avg", SM_STALE_HIGH, Other);
  mt_v_bat_pack_cstddev = new OvmsMetricFloat("xse.v.b.p.capacity.stddev", SM_STALE_HIGH, Other);
  mt_v_bat_pack_cstddev_max = new OvmsMetricFloat("xse.v.b.p.capacity.stddev.max", SM_STALE_HIGH, Other);

  mt_v_bat_cell_capacity = new OvmsMetricVector<float>("xse.v.b.c.capacity", SM_STALE_HIGH, Other);
  mt_v_bat_cell_cmin = new OvmsMetricVector<float>("xse.v.b.c.capacity.min", SM_STALE_HIGH, Other);
  mt_v_bat_cell_cmax = new OvmsMetricVector<float>("xse.v.b.c.capacity.max", SM_STALE_HIGH, Other);
  mt_v_bat_cell_cdevmax = new OvmsMetricVector<float>("xse.v.b.c.capacity.dev.max", SM_STALE_HIGH, Other);
  
  mt_v_bat_HVoff_time = new OvmsMetricInt("xse.v.b.p.hv.off.time", SM_STALE_HIGH, Seconds);
  mt_v_bat_HV_lowcurrent = new OvmsMetricInt("xse.v.b.p.hv.lowcurrent", SM_STALE_HIGH, Seconds);
  mt_v_bat_OCVtimer = new OvmsMetricInt("xse.v.b.p.ocv.timer", SM_STALE_HIGH, Seconds);
  mt_v_bat_SOH = new OvmsMetricInt("xse.v.b.p.soh", SM_STALE_HIGH, Other);
  mt_v_bat_LastMeas_days = new OvmsMetricInt("xse.v.b.p.last.meas.days", SM_STALE_HIGH, Other);
  mt_v_bat_Cap_meas_quality = new OvmsMetricFloat("xse.v.b.p.capacity.quality", SM_STALE_HIGH, Other);
  mt_v_bat_Cap_combined_quality = new OvmsMetricFloat("xse.v.b.p.capacity.combined.quality", SM_STALE_HIGH, Other);
  mt_v_bat_Cap_As_min = new OvmsMetricInt("xse.v.b.p.capacity.as.minimum", SM_STALE_HIGH, Other);
  mt_v_bat_Cap_As_max = new OvmsMetricInt("xse.v.b.p.capacity.as.maximum", SM_STALE_HIGH, Other);
  mt_v_bat_Cap_As_avg = new OvmsMetricInt("xse.v.b.p.capacity.as.average", SM_STALE_HIGH, Other);
  
  BmsSetCellArrangementCapacity(93,3);
  
  // init poller:
  PollSetPidList(m_can1, smarted_polls);
  PollSetState(0);
}

/**
 * Incoming poll reply messages
 */
void OvmsVehicleSmartED::IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t remain) {
  static string rxbuf;
  static uint16_t last_pid = -1;
  
  if (pid != last_pid) {
    ESP_LOGD(TAG, "pid: %04x length: %d m_poll_ml_remain: %d m_poll_ml_frame: %d", pid, length, m_poll_ml_remain, m_poll_ml_frame);
    last_pid = pid;
    m_poll_ml_frame=0;
  }
  
  // init / fill rx buffer:
  if (m_poll_ml_frame == 0) {
    rxbuf.clear();
    rxbuf.reserve(length + remain);
  }
  rxbuf.append((char*)data, length);
  
  if (pid == 0xF111 && m_poll_ml_frame == 1) {
    remain=0;
    ESP_LOGD(TAG, "End pid: %04x length: %d m_poll_ml_remain: %d m_poll_ml_frame: %d", pid, length, m_poll_ml_remain, m_poll_ml_frame);
  }
  
  if ((data[5] * 256 + data[6]) == 21845) {
    remain=0;
    ESP_LOGD(TAG, "End pid: %04x length: %d m_poll_ml_remain: %d m_poll_ml_frame: %d", pid, length, m_poll_ml_remain, m_poll_ml_frame);
  }
  
  if (remain)
    return;
  
  // complete:
  switch (pid) {
    case 0x0201: // rqBattTemperatures
      PollReply_BMS_BattTemp(rxbuf.data(), rxbuf.size());
      break;
    case 0x0202: // rqBattModuleTemperatures
      PollReply_BMS_ModuleTemp(rxbuf.data(), rxbuf.size());
      break;
    case 0x0208: // rqBattVolts
      PollReply_BMS_BattVolts(rxbuf.data(), rxbuf.size());
      break;
    case 0x0310: // rqBattCapacity
      PollReply_BMS_BattCapacity(rxbuf.data(), rxbuf.size());
      break;
    case 0xF111: // rqChargerPN_HW
      PollReply_NLG6_ChargerPN_HW(rxbuf.data(), rxbuf.size());
      break;
    case 0x0226: // rqChargerVoltages
      PollReply_NLG6_ChargerVoltages(rxbuf.data(), rxbuf.size());
      break;
    case 0x0225: // rqChargerAmps
      PollReply_NLG6_ChargerAmps(rxbuf.data(), rxbuf.size());
      break;
    case 0x022A: // rqChargerSelCurrent
      PollReply_NLG6_ChargerSelCurrent(rxbuf.data(), rxbuf.size());
      break;
    case 0x0223: // rqChargerTemperatures
      PollReply_NLG6_ChargerTemperatures(rxbuf.data(), rxbuf.size());
      break;
    // Unknown: output
    default: {
      char *buf = NULL;
      size_t rlen = rxbuf.size(), offset = 0;
      do {
        rlen = FormatHexDump(&buf, rxbuf.data() + offset, rlen, 16);
        offset += 16;
        ESP_LOGW(TAG, "OBD2: unhandled reply [%02x %02x]: %s", type, pid, buf ? buf : "-");
      } while (rlen);
      if (buf)
        free(buf);
      break;
    }
  }
}

void OvmsVehicleSmartED::PollReply_BMS_BattTemp(const char* reply_data, uint16_t reply_len) {
  int16_t Temps[7];
  int n;
  
  for(n = 0; n < (7 * 2); n = n + 2){
    Temps[n/2] = ((reply_data[n + 1] * 256 + reply_data[n + 2]));
  }
  if (Temps[0]/64 != -512)
    StandardMetrics.ms_v_bat_temp->SetValue(Temps[0]/64);
}

void OvmsVehicleSmartED::PollReply_BMS_ModuleTemp(const char* reply_data, uint16_t reply_len) {
  int16_t Temps[13];
  int n;
  int i;

  for(n = 0; n < (9 * 2); n = n + 2){
    Temps[n/2] = ((reply_data[n + 1] * 256 + reply_data[n + 2]));
  }
  for (n = 0; n < 9; n = n + 3) {
    for (i = 0; i < 3; i++) {
      if (i==2) BmsSetCellTemperature(n/3, (float) Temps[n + i]/64);
    }
  }
}

void OvmsVehicleSmartED::PollReply_BMS_BattVolts(const char* reply_data, uint16_t reply_len) {
  int n;
  
  for(n = 0; n < (CELLCOUNT * 2); n = n + 2){
    float Cells = (reply_data[n + 1] * 256 + reply_data[n + 2]);
    BmsSetCellVoltage(n/2, Cells/1000);
  } 
}

void OvmsVehicleSmartED::PollReply_BMS_BattCapacity(const char* reply_data, uint16_t reply_len) {
  for(uint16_t n = 0; n < (CELLCOUNT * 2); n = n + 2){
    BmsSetCellCapacity(n/2, (reply_data[n + 22] * 256 + reply_data[n + 23]));
  }
/*
  myBMS->Ccap_As.min = CellCapacity.minimum(&myBMS->CAP_min_at);
  myBMS->Ccap_As.max = CellCapacity.maximum(&myBMS->CAP_max_at);
  myBMS->Ccap_As.mean = CellCapacity.mean();
*/
  mt_v_bat_HVoff_time->SetValue( reply_data[2] * 65535 + (uint16_t) reply_data[3] * 256 + reply_data[4] );
  mt_v_bat_HV_lowcurrent->SetValue( (unsigned long) reply_data[6] * 65535 + (uint16_t) reply_data[7] * 256 + reply_data[8] );
  mt_v_bat_OCVtimer->SetValue( (uint16_t) reply_data[9] * 256 + reply_data[10] );
  mt_v_bat_SOH->SetValue( reply_data[11] );
  mt_v_bat_Cap_As_min->SetValue( reply_data[18] * 256 + reply_data[19] );
  mt_v_bat_Cap_As_avg->SetValue( reply_data[20] * 256 + reply_data[21] );
  mt_v_bat_Cap_As_max->SetValue( reply_data[14] * 256 + reply_data[15] );
  mt_v_bat_LastMeas_days->SetValue( reply_data[424] * 256 + reply_data[425] );
  uint16_t value;
  value = reply_data[426] * 256 + reply_data[427];
  mt_v_bat_Cap_meas_quality->SetValue( value / 65535.0 );
  value = reply_data[422] * 256 + reply_data[423];
  mt_v_bat_Cap_combined_quality->SetValue( value / 65535.0 );
  
}

void OvmsVehicleSmartED::PollReply_NLG6_ChargerPN_HW(const char* reply_data, uint16_t reply_len) {
  int n;
  int comp = 0;
  char NLG6PN_HW[11];
  memset(NLG6PN_HW, 0, sizeof(NLG6PN_HW));
  
  for (n = 1; n < 11; n++) {
    NLG6PN_HW[n - 1] = reply_data[n];
    if (reply_data[n] == NLG6_PN_HW[n - 1]) {
      comp++;
    }
  }
  mt_nlg6_pn_hw->SetValue(NLG6PN_HW);
  
  if (comp == 10){
    mt_nlg6_present->SetValue(true);
  } else {
    mt_nlg6_present->SetValue(false);
  }
}

void OvmsVehicleSmartED::PollReply_NLG6_ChargerVoltages(const char* reply_data, uint16_t reply_len) {
  float NLG6MainsVoltage[3];
  
  if (mt_nlg6_present->AsBool()){
    mt_nlg6_dc_lv->SetValue(reply_data[5]/10.0);
    mt_nlg6_dc_hv->SetValue((reply_data[6] * 256 + reply_data[7])/10.0);
    NLG6MainsVoltage[0] = (reply_data[8] * 256 + reply_data[9])/10.0;
    NLG6MainsVoltage[1] = (reply_data[10] * 256 + reply_data[11])/10.0;
    NLG6MainsVoltage[2] = (reply_data[12] * 256 + reply_data[13])/10.0;
  } else {
    mt_nlg6_dc_lv->SetValue(reply_data[3]/10.0);
    if ((reply_data[6] * 256 + reply_data[7]) != 8190) {  //OBL showing only valid data while charging
      mt_nlg6_dc_hv->SetValue((reply_data[6] * 256 + reply_data[7])/10.0);
    } else {
      mt_nlg6_dc_hv->SetValue(0);
    }
    if ((reply_data[8] * 256 + reply_data[9]) != 8190) {  //OBL showing only valid data while charging
      NLG6MainsVoltage[0] = (reply_data[8] * 256 + reply_data[9])/10.0;
    } else {
      NLG6MainsVoltage[0] = 0;
    }
    NLG6MainsVoltage[1] = 0;
    NLG6MainsVoltage[2] = 0;
  }
  mt_nlg6_main_volts->SetElemValues(0, 3, NLG6MainsVoltage);
  if (NLG6MainsVoltage[0] != 0) StandardMetrics.ms_v_charge_voltage->SetValue(NLG6MainsVoltage[0]);
}

void OvmsVehicleSmartED::PollReply_NLG6_ChargerAmps(const char* reply_data, uint16_t reply_len) {
  float NLG6MainsAmps[3]; 
  
  if (mt_nlg6_present->AsBool()){
    mt_nlg6_dc_current->SetValue((reply_data[1] * 256 + reply_data[2])/10.0); 
    NLG6MainsAmps[0] = (reply_data[3] * 256 + reply_data[4])/10.0;  
    NLG6MainsAmps[1] = (reply_data[5] * 256 + reply_data[6])/10.0;
    NLG6MainsAmps[2] = (reply_data[7] * 256 + reply_data[8])/10.0; 
    mt_nlg6_amps_chargingpoint->SetValue((reply_data[13] * 256 + reply_data[14])/10.0);
    mt_nlg6_amps_cablecode->SetValue((reply_data[15] * 256 + reply_data[16])/10.0);
  } else {
    if ((reply_data[3] * 256 + reply_data[4]) != 2047) {  //OBL showing only valid data while charging
      NLG6MainsAmps[0] = (reply_data[3] * 256 + reply_data[4])/10.0;
    } else {
      NLG6MainsAmps[0] = 0;
    }
    NLG6MainsAmps[1] = 0;
    NLG6MainsAmps[2] = 0;
    mt_nlg6_amps_chargingpoint->SetValue(0);
    if ((reply_data[15] * 256 + reply_data[16]) != 2047) {  //OBL showing only valid data while charging
      mt_nlg6_dc_current->SetValue((reply_data[15] * 256 + reply_data[16])/10.0);
    } else {
      mt_nlg6_dc_current->SetValue(0);
    }
    //Usable AmpsCode from Cable seem to be also a word with OBL as with NLG6?!
    mt_nlg6_amps_cablecode->SetValue((reply_data[5] * 256 + reply_data[6])/10.0);
    //NLG6AmpsCableCode = data[12]; //12
  }
  mt_nlg6_main_amps->SetElemValues(0, 3, NLG6MainsAmps);
  if (NLG6MainsAmps[0] != 0) StandardMetrics.ms_v_charge_current->SetValue(NLG6MainsAmps[0]);
}

void OvmsVehicleSmartED::PollReply_NLG6_ChargerSelCurrent(const char* reply_data, uint16_t reply_len) {
  if(mt_nlg6_present->AsBool()){
    mt_nlg6_amps_setpoint->SetValue(reply_data[5]); //Get data for NLG6 fast charger
  } else {
    mt_nlg6_amps_setpoint->SetValue(reply_data[4]); //7 //Get data for standard OBL
  }
}

void OvmsVehicleSmartED::PollReply_NLG6_ChargerTemperatures(const char* reply_data, uint16_t reply_len) {
  float NLG6Temps[9];
  int n;
  
  if (mt_nlg6_present->AsBool()){
    mt_nlg6_temp_coolingplate->SetValue((reply_data[1] < 0xFF) ? reply_data[1]-40 : 0);
    for(n = 0; n < 8; n++) {
      NLG6Temps[n] = (reply_data[n + 2] < 0xFF) ? reply_data[n + 2]-40 : 0;
    }
    mt_nlg6_temp_reported->SetValue((reply_data[9] < 0xFF) ? reply_data[9]-40 : 0);
    mt_nlg6_temp_socket->SetValue((reply_data[10] < 0xFF) ? reply_data[10]-40 : 0);
    mt_nlg6_temps->SetElemValues(0, 8, NLG6Temps);
  } else {
    mt_nlg6_temp_coolingplate->SetValue((reply_data[2] < 0xFF) ? reply_data[2]-40 : 0); //5
    mt_nlg6_temp_reported->SetValue((reply_data[4] < 0xFF) ? reply_data[4]-40 : 0); //7
    mt_nlg6_temp_socket->SetValue((reply_data[6] < 0xFF) ? reply_data[6]-40 : 0); //9
  }
  if (mt_nlg6_temp_reported->AsFloat() != 0) StandardMetrics.ms_v_charge_temp->SetValue(mt_nlg6_temp_reported->AsFloat());
}

// BMS helpers
void OvmsVehicleSmartED::BmsSetCellArrangementCapacity(int readings, int readingspermodule) {
  if (m_bms_capacitys != NULL) delete m_bms_capacitys;
  m_bms_capacitys = new float[readings];
  if (m_bms_cmins != NULL) delete m_bms_cmins;
  m_bms_cmins = new float[readings];
  if (m_bms_cmaxs != NULL) delete m_bms_cmaxs;
  m_bms_cmaxs = new float[readings];
  if (m_bms_cdevmaxs != NULL) delete m_bms_cdevmaxs;
  m_bms_cdevmaxs = new float[readings];

  m_bms_bitset_c.clear();
  m_bms_bitset_c.reserve(readings);

  m_bms_readings_c = readings;
  m_bms_readingspermodule_c = readingspermodule;

  BmsResetCellCapacitys();
}
  
void OvmsVehicleSmartED::BmsSetCellCapacity(int index, float value) {
  // ESP_LOGI(TAG,"BmsSetCellCapacity(%d,%f) c=%d", index, value, m_bms_bitset_cc);
  if ((index<0)||(index>=m_bms_readings_c)) return;
  if ((value<m_bms_limit_cmin)||(value>m_bms_limit_cmax)) return;
  m_bms_capacitys[index] = value;

  if (! m_bms_has_capacitys)
    {
    m_bms_cmins[index] = value;
    m_bms_cmaxs[index] = value;
    }
  else if (m_bms_cmins[index] > value)
    m_bms_cmins[index] = value;
  else if (m_bms_cmaxs[index] < value)
    m_bms_cmaxs[index] = value;

  if (m_bms_bitset_c[index] == false) m_bms_bitset_cc++;
  if (m_bms_bitset_cc == m_bms_readings_c) {
    // get min, max, avg & standard deviation:
    double sum=0, sqrsum=0, avg, stddev=0;
    float min=0, max=0;
    for (int i=0; i<m_bms_readings_c; i++) {
      sum += m_bms_capacitys[i];
      sqrsum += SQR(m_bms_capacitys[i]);
      if (min==0 || m_bms_capacitys[i]<min)
        min = m_bms_capacitys[i];
      if (max==0 || m_bms_capacitys[i]>max)
        max = m_bms_capacitys[i];
    }
    avg = sum / m_bms_readings_c;
    stddev = sqrt(LIMIT_MIN((sqrsum / m_bms_readings_c) - SQR(avg), 0));
    // check cell deviations:
    float dev;
    for (int i=0; i<m_bms_readings_c; i++) {
      dev = ROUNDPREC(m_bms_capacitys[i] - avg, 1);
      if (ABS(dev) > ABS(m_bms_cdevmaxs[i]))
        m_bms_cdevmaxs[i] = dev;
    }
    // publish to metrics:
    avg = ROUNDPREC(avg, 1);
    stddev = ROUNDPREC(stddev, 1);
    mt_v_bat_pack_cmin->SetValue(min);
    mt_v_bat_pack_cmax->SetValue(max);
    mt_v_bat_pack_cavg->SetValue(avg);
    mt_v_bat_pack_cstddev->SetValue(stddev);
    if (stddev > mt_v_bat_pack_cstddev_max->AsFloat())
      mt_v_bat_pack_cstddev_max->SetValue(stddev);
    mt_v_bat_cell_capacity->SetElemValues(0, m_bms_readings_c, m_bms_capacitys);
    mt_v_bat_cell_cmin->SetElemValues(0, m_bms_readings_c, m_bms_cmins);
    mt_v_bat_cell_cmax->SetElemValues(0, m_bms_readings_c, m_bms_cmaxs);
    mt_v_bat_cell_cdevmax->SetElemValues(0, m_bms_readings_c, m_bms_cdevmaxs);
    // complete:
    m_bms_has_capacitys = true;
    m_bms_bitset_c.clear();
    m_bms_bitset_c.resize(m_bms_readings_c);
    m_bms_bitset_cc = 0;
  }
  else {
    m_bms_bitset_c[index] = true;
  }
}

void OvmsVehicleSmartED::BmsRestartCellCapacitys() {
  m_bms_bitset_c.clear();
  m_bms_bitset_c.resize(m_bms_readings_c);
  m_bms_bitset_cc = 0;
}

void OvmsVehicleSmartED::BmsResetCellCapacitys() {
  if (m_bms_readings_c > 0) {
    m_bms_bitset_c.clear();
    m_bms_bitset_c.resize(m_bms_readings_c);
    m_bms_bitset_cc = 0;
    m_bms_has_capacitys = false;
    for (int k=0; k<m_bms_readings_c; k++) {
      m_bms_cmins[k] = 0;
      m_bms_cmaxs[k] = 0;
      m_bms_cdevmaxs[k] = 0;
    }
    mt_v_bat_cell_cmin->ClearValue();
    mt_v_bat_cell_cmax->ClearValue();
    mt_v_bat_cell_cdevmax->ClearValue();
    mt_v_bat_pack_cstddev_max->SetValue(mt_v_bat_pack_cstddev->AsFloat());
  }
}
