/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          1th October 2018
 ;
 ;    Changes:
 ;    1.0  Initial release
 ;
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
 ; https://github.com/MyLab-odyssey/ED4scan
 */
 
#include "ovms_log.h"
static const char *TAG = "v-smart_1";

#include <stdio.h>
#include "vehicle_smart_1.h"

static const OvmsPoller::poll_pid_t smart_1_polls[] =
{
  // Note: poller ticker cycles at 3600 seconds = max period
  // { tx, rx, type, pid, {OFF,AWAKE,ON,CHARGING}, bus, protocol }
  { 0x1DD01635, 0x1EC6AE80, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x4801, {  0,10,60,60 }, 0, ISOTP_EXTFRAME },
  { 0x1DD01635, 0x1EC6AE80, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x4802, {  0,10,60,60 }, 0, ISOTP_EXTFRAME },
  { 0x1DD01635, 0x1EC6AE80, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x4803, {  0,10,60,60 }, 0, ISOTP_EXTFRAME },
  { 0x1DD01635, 0x1EC6AE80, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x496D, {  0,60,60,60 }, 0, ISOTP_EXTFRAME },
  { 0x1DD01635, 0x1EC6AE80, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x491B, {  0,60,60,60 }, 0, ISOTP_EXTFRAME },
  { 0x1DD01635, 0x1EC6AE80, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xDD01, {  0,60,60,60 }, 0, ISOTP_EXTFRAME },
  POLL_LIST_END
};

OvmsVehicleSmart_1::OvmsVehicleSmart_1()
  {
  ESP_LOGI(TAG, "Start smart #1 vehicle module");
  
  MyConfig.RegisterParam("xsht", "Smart #1", true, true);

  RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);

  ConfigChanged(NULL);
  
  // Poll Specific PIDs
  POLLSTATE_OFF;
  PollSetPidList(m_can1, smart_1_polls);
  PollSetThrottling(5);
  PollSetResponseSeparationTime(20);
  }

/**
 * ConfigChanged: reload single/all configuration variables
 */
void OvmsVehicleSmart_1::ConfigChanged(OvmsConfigParam* param)
  {
  if (param && param->GetName() != "xsht")
    return;

  ESP_LOGD(TAG, "Smart #1 reload configuration");

  m_enable_write    = MyConfig.GetParamValueBool("xsht", "canwrite", false);

  if (!m_enable_write) POLLSTATE_OFF;
  }

void OvmsVehicleSmart_1::IncomingFrameCan1(CAN_frame_t* p_frame) {
  if (m_candata_poll != 1) {
    ESP_LOGI(TAG,"Car has woken (CAN bus activity)");
    StandardMetrics.ms_v_env_awake->SetValue(true);
    m_candata_poll = 1;
    if (m_enable_write) POLLSTATE_ON;
  }
}

/**
 * Incoming poll reply messages
 */
void OvmsVehicleSmart_1::IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length) {
  string& rxbuf = sht1_obd_rxbuf;

  if (job.pid != m_last_pid) {
    //ESP_LOGD(TAG, "pid: %04x length: %d mlremain: %d mlframe: %d", pid, length, mlremain, mlframe);
    // If this is not the first frame .. ignore it until we get one.
    if (job.mlframe > 0)
      return;
    m_last_pid = job.pid;
  }

  // init / fill rx buffer:
  if (job.mlframe == 0) {
    rxbuf.clear();
    rxbuf.reserve(length + job.mlremain);
  }
  rxbuf.append((char*)data, length);

  if (job.mlremain)
    return;

  // complete:
  switch (job.pid) {
    case 0x4801: // soc
      PollReply_SOC(rxbuf.data(), rxbuf.size());
      break;
    case 0x4802: // current
      PollReply_HV_cur(rxbuf.data(), rxbuf.size());
      break;
    case 0x4803: // voltage
      PollReply_HV_volt(rxbuf.data(), rxbuf.size());
      break;
    case 0x496D: // SOH
      PollReply_SOH(rxbuf.data(), rxbuf.size());
      break;
    case 0x491B: // AVG batt temp
      PollReply_bat_temp(rxbuf.data(), rxbuf.size());
      break;
    case 0xDD01: // odo
      PollReply_odo(rxbuf.data(), rxbuf.size());
      break;

    // Unknown: output
    default: {
      char *buf = NULL;
      size_t rlen = rxbuf.size(), offset = 0;
      do {
        rlen = FormatHexDump(&buf, rxbuf.data() + offset, rlen, 16);
        offset += 16;
        ESP_LOGW(TAG, "OBD2: unhandled reply [%02x %02x]: %s", job.type, job.pid, buf ? buf : "-");
      } while (rlen);
      if (buf)
        free(buf);
      break;
    }
  }

}

void OvmsVehicleSmart_1::IncomingPollError(const OvmsPoller::poll_job_t &job, int32_t code) {
  switch (job.moduleid_rec) {
    default:
      ESP_LOGE(TAG, "IncomingPollError: PID %04X: err=%02X", (unsigned)job.pid, code);
      break;
  }
}

void OvmsVehicleSmart_1::Ticker1(uint32_t ticker) {
  float volt = StandardMetrics.ms_v_bat_12v_voltage->AsFloat(0.0f);
  
  if(volt >= 13.5f) {
    m_candata_timer = SHT1_CANDATA_TIMEOUT;
    if (m_candata_poll != 1) {
      ESP_LOGI(TAG,"Car has woken (CAN bus activity)");
      StandardMetrics.ms_v_env_awake->SetValue(true);
      m_candata_poll = 1;
      if (m_enable_write) POLLSTATE_ON;
    }
  }
  if (m_candata_timer > 0) {
    if (--m_candata_timer == 0) {
      // Car has gone to sleep
      ESP_LOGI(TAG,"Car has gone to sleep (CAN bus timeout)");
      StandardMetrics.ms_v_env_awake->SetValue(false);
      POLLSTATE_OFF;
      m_candata_poll = 0;
    }
  }
}

void OvmsVehicleSmart_1::PollReply_SOC(const char* data, uint16_t reply_len) {
  StandardMetrics.ms_v_bat_soc->SetValue((float) (0.002f*(CAN_UINT(0))-3)/94*100);
}

void OvmsVehicleSmart_1::PollReply_HV_cur(const char* data, uint16_t reply_len) {
  StandardMetrics.ms_v_bat_current->SetValue((float) 0.1f*(CAN_UINT(0)-16384)); // "equation": "0.1*((B*256+C)-16384)", // A is always 02
}

void OvmsVehicleSmart_1::PollReply_HV_volt(const char* data, uint16_t reply_len) {
  StandardMetrics.ms_v_bat_voltage->SetValue((float) 0.01f*CAN_UINT(0));
}

void OvmsVehicleSmart_1::PollReply_SOH(const char* data, uint16_t reply_len) {
  StandardMetrics.ms_v_bat_soh->SetValue((float) CAN_UINT(2)*0.01f);
}

void OvmsVehicleSmart_1::PollReply_bat_temp(const char* data, uint16_t reply_len) {
  StandardMetrics.ms_v_bat_temp->SetValue((float) (CAN_UINT(0)-5000)/100);
}

void OvmsVehicleSmart_1::PollReply_odo(const char* data, uint16_t reply_len) {
  StandardMetrics.ms_v_pos_odometer->SetValue((float) CAN_UINT24(0));
}
OvmsVehicleSmart_1::~OvmsVehicleSmart_1()
  {
  ESP_LOGI(TAG, "Shutdown smart #1 vehicle module");
  }

class OvmsVehicleSmart_1Init
  {
  public: OvmsVehicleSmart_1Init();
} MyOvmsVehicleSmart_1Init  __attribute__ ((init_priority (9000)));

OvmsVehicleSmart_1Init::OvmsVehicleSmart_1Init()
  {
  ESP_LOGI(TAG, "Registering Vehicle: smart #1 (9000)");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleSmart_1>("SHT1","Smart #1");
  }
