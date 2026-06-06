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

#ifndef __VEHICLE_SMART_1_H__
#define __VEHICLE_SMART_1_H__

#define VERSION "1.0.0"
#define PRESET_VERSION 20260606 // Configuration preset version

#include "vehicle.h"

// CAN buffer access macros: b=byte# 0..7 / n=nibble# 0..15
#define CAN_BYTE(b)     data[b]
#define CAN_UINT(b)     (((UINT)CAN_BYTE(b) << 8) | CAN_BYTE(b+1))
#define CAN_UINT24(b)   (((uint32_t)CAN_BYTE(b) << 16) | ((UINT)CAN_BYTE(b+1) << 8) | CAN_BYTE(b+2))
#define CAN_UINT32(b)   (((uint32_t)CAN_BYTE(b) << 24) | ((uint32_t)CAN_BYTE(b+1) << 16)  | ((UINT)CAN_BYTE(b+2) << 8) | CAN_BYTE(b+3))
#define CAN_NIBL(b)     (data[b] & 0x0f)
#define CAN_NIBH(b)     (data[b] >> 4)
#define CAN_NIB(n)      (((n)&1) ? CAN_NIBL((n)>>1) : CAN_NIBH((n)>>1))

#define POLLSTATE_OFF					PollSetState(0);
#define POLLSTATE_ON					PollSetState(1);
#define POLLSTATE_RUNNING			PollSetState(2);
#define POLLSTATE_CHARGING		PollSetState(3);

#define SHT1_CANDATA_TIMEOUT 10

using namespace std;

class OvmsVehicleSmart_1 : public OvmsVehicle
  {
  public:
    OvmsVehicleSmart_1();
    ~OvmsVehicleSmart_1();

  public:
    void IncomingFrameCan1(CAN_frame_t* p_frame) override;
    void IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length) override;
    void IncomingPollError(const OvmsPoller::poll_job_t &job, int32_t code) override;

  public:
    void ConfigChanged(OvmsConfigParam* param) override;

  private:
    unsigned int m_candata_timer;
    unsigned int m_candata_poll;

  protected:
    string sht1_obd_rxbuf;
    uint16_t m_last_pid = 0;
    bool m_enable_write;                    // canwrite
    void Ticker1(uint32_t ticker) override;
    void PollReply_SOC(const char* data, uint16_t reply_len);
    void PollReply_HV_cur(const char* data, uint16_t reply_len);
    void PollReply_HV_volt(const char* data, uint16_t reply_len);
    void PollReply_SOH(const char* data, uint16_t reply_len);
    void PollReply_bat_temp(const char* data, uint16_t reply_len);
    void PollReply_odo(const char* data, uint16_t reply_len);
  };

#endif //#ifndef __VEHICLE_SMART_1_H__