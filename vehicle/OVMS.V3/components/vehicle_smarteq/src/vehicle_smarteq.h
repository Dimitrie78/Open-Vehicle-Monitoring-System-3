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
 */

#ifndef __VEHICLE_SMARTEQ_H__
#define __VEHICLE_SMARTEQ_H__

#include <atomic>

#include "can.h"
#include "vehicle.h"

#include "ovms_log.h"
#include "ovms_config.h"
#include "ovms_metrics.h"
#include "ovms_command.h"
#include "freertos/timers.h"
#ifdef CONFIG_OVMS_COMP_WEBSERVER
#include "ovms_webserver.h"
#endif

// CAN buffer access macros: b=byte# 0..7 / n=nibble# 0..15
#define CAN_BYTE(b)     data[b]
#define CAN_UINT(b)     (((UINT)CAN_BYTE(b) << 8) | CAN_BYTE(b+1))
#define CAN_UINT24(b)   (((uint32_t)CAN_BYTE(b) << 16) | ((UINT)CAN_BYTE(b+1) << 8) | CAN_BYTE(b+2))
#define CAN_UINT32(b)   (((uint32_t)CAN_BYTE(b) << 24) | ((uint32_t)CAN_BYTE(b+1) << 16)  | ((UINT)CAN_BYTE(b+2) << 8) | CAN_BYTE(b+3))
#define CAN_NIBL(b)     (data[b] & 0x0f)
#define CAN_NIBH(b)     (data[b] >> 4)
#define CAN_NIB(n)      (((n)&1) ? CAN_NIBL((n)>>1) : CAN_NIBH((n)>>1))

using namespace std;

typedef std::vector<OvmsPoller::poll_pid_t, ExtRamAllocator<OvmsPoller::poll_pid_t>> poll_vector_t;
typedef std::initializer_list<const OvmsPoller::poll_pid_t> poll_list_t;

class OvmsVehicleSmartEQ : public OvmsVehicle
{
  public:
    OvmsVehicleSmartEQ();
    ~OvmsVehicleSmartEQ();

  public:
    void IncomingFrameCan1(CAN_frame_t* p_frame) override;
    void IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length) override;
    void IncomingPollError(const OvmsPoller::poll_job_t &job, uint16_t code) override;
    void HandleCharging();
    void HandleEnergy();
    void UpdateChargeMetrics();
    int  calcMinutesRemaining(float target, float charge_voltage, float charge_current);
    void HandlePollState();
    void OnlineState();
    void ObdModifyPoll();
    void ResetChargingValues();
    void ResetTripCounters();
    void ResetTotalCounters();

public:
    vehicle_command_t CommandClimateControl(bool enable) override;
    vehicle_command_t CommandHomelink(int button, int durationms=1000) override;
    vehicle_command_t CommandWakeup() override;
    vehicle_command_t CommandStat(int verbosity, OvmsWriter* writer) override;

  public:
#ifdef CONFIG_OVMS_COMP_WEBSERVER
    void WebInit();
    void WebDeInit();
    static void WebCfgFeatures(PageEntry_t& p, PageContext_t& c);
    static void WebCfgBattery(PageEntry_t& p, PageContext_t& c);
#endif
    void ConfigChanged(OvmsConfigParam* param) override;
    bool SetFeature(int key, const char* value);
    const std::string GetFeature(int key);
    uint64_t swap_uint64(uint64_t val);

  private:
    unsigned int m_candata_timer;
    unsigned int m_candata_poll;
    bool m_charge_start;

  protected:
    void Ticker1(uint32_t ticker) override;
    void PollerStateTicker(canbus *bus) override;
    void GetDashboardConfig(DashboardConfig& cfg);
    virtual void CalculateEfficiency();
    void vehicle_smart_car_on(bool isOn);
    
    void PollReply_BMS_BattVolts(const char* data, uint16_t reply_len, uint16_t start);
    void PollReply_BMS_BattTemps(const char* data, uint16_t reply_len);
    void PollReply_BMS_BattState(const char* data, uint16_t reply_len);
    void PollReply_HVAC(const char* data, uint16_t reply_len);
    void PollReply_TDB(const char* data, uint16_t reply_len);
    void PollReply_VIN(const char* data, uint16_t reply_len);
    void PollReply_EVC_HV_Energy(const char* data, uint16_t reply_len);
    void PollReply_EVC_DCDC_State(const char* data, uint16_t reply_len);
    void PollReply_EVC_DCDC_Load(const char* data, uint16_t reply_len);
    void PollReply_EVC_DCDC_Amps(const char* data, uint16_t reply_len);
    void PollReply_EVC_DCDC_Power(const char* data, uint16_t reply_len);
    void PollReply_OBL_ChargerAC(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Ph1_RMS_A(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Ph2_RMS_A(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Ph3_RMS_A(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Ph12_RMS_V(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Ph23_RMS_V(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Ph31_RMS_V(const char* data, uint16_t reply_len);
    void PollReply_OBL_JB2AC_Power(const char* data, uint16_t reply_len);

  protected:
    bool m_enable_write;                    // canwrite
    bool m_enable_LED_state;                // Online LED State
    bool m_ios_tpms_fix;                    // IOS TPMS Display Fix
    bool m_resettrip;                       // Reset Trip Values when Charging/Driving
    bool m_resettotal;                      // Reset kWh/100km Values when Driving

    #define DEFAULT_BATTERY_CAPACITY 17600
    #define MAX_POLL_DATA_LEN 126
    #define CELLCOUNT 96
    #define SQ_CANDATA_TIMEOUT 10

  protected:
    std::string   m_rxbuf;

  protected:
    OvmsMetricVector<float> *mt_bms_temps;              // BMS temperatures
    OvmsMetricBool          *mt_bus_awake;              // Can Bus active
    OvmsMetricFloat         *mt_use_at_reset;           // kWh use at reset in Display
    OvmsMetricFloat         *mt_pos_odometer_start;     // remind odometer start
    OvmsMetricFloat         *mt_pos_odometer_start_total;     // remind odometer start for kWh/100km
    OvmsMetricFloat         *mt_pos_odometer_trip_total;// counted km for kWh/100km
    OvmsMetricBool          *mt_obl_fastchg;            // ODOmeter at Start
    OvmsMetricFloat         *mt_evc_hv_energy;          //!< available energy in kWh
    OvmsMetricFloat         *mt_evc_LV_DCDC_amps;       //!< current of DC/DC LV system, not 12V battery!
    OvmsMetricFloat         *mt_evc_LV_DCDC_load;       //!< load in % of DC/DC LV system, not 12V battery!
    OvmsMetricFloat         *mt_evc_LV_DCDC_power;      //!< power in W (x/10) of DC/DC output of LV system, not 12V battery!
    OvmsMetricInt           *mt_evc_LV_DCDC_state;      //!< DC/DC state
    OvmsMetricFloat         *mt_bms_CV_Range_min;       //!< minimum cell voltage in V, no offset
    OvmsMetricFloat         *mt_bms_CV_Range_max;       //!< maximum cell voltage in V, no offset
    OvmsMetricFloat         *mt_bms_CV_Range_mean;      //!< average cell voltage in V, no offset
    OvmsMetricFloat         *mt_bms_BattLinkVoltage;    //!< Link voltage to drivetrain inverter
    OvmsMetricFloat         *mt_bms_BattCV_Sum;         //!< Sum of single cell voltages
    OvmsMetricFloat         *mt_bms_BattPower_voltage;  //!< voltage value sample (raw), (x/64) for actual value
    OvmsMetricFloat         *mt_bms_BattPower_current;  //!< current value sample (raw), (x/32) for actual value
    OvmsMetricFloat         *mt_bms_BattPower_power;    //!< calculated power of sample in kW
    OvmsMetricInt           *mt_bms_HVcontactState;     //!< contactor state: 0 := OFF, 1 := PRECHARGE, 2 := ON
    OvmsMetricFloat         *mt_bms_HV;                 //!< total voltage of HV system in V
    OvmsMetricInt           *mt_bms_EVmode;             //!< Mode the EV is actually in: 0 = none, 1 = slow charge, 2 = fast charge, 3 = normal, 4 = fast balance
    OvmsMetricFloat         *mt_bms_LV;                 //!< 12V onboard voltage / LV system
    OvmsMetricFloat         *mt_bms_Amps;               //!< battery current in ampere (x/32) reported by by BMS
    OvmsMetricFloat         *mt_bms_Amps2;              //!< battery current in ampere read by live data on CAN or from BMS
    OvmsMetricFloat         *mt_bms_Power;              //!< power as product of voltage and amps in kW
    OvmsMetricVector<float> *mt_obl_main_amps;          //!< AC current of L1, L2, L3
    OvmsMetricVector<float> *mt_obl_main_volts;         //!< AC voltage of L1, L2, L3
    OvmsMetricVector<float> *mt_obl_main_CHGpower;      //!< Power of rail1, rail2 W (x/2) & max available kw (x/64)
    OvmsMetricFloat         *mt_obl_main_freq;          //!< AC input frequency

  protected:
    bool m_booster_start;
    int m_led_state;
  
  protected:
    poll_vector_t       m_poll_vector;              // List of PIDs to poll
    int                 m_cfg_cell_interval_drv;    // Cell poll interval while driving, default 15 sec.
    int                 m_cfg_cell_interval_chg;    // … while charging, default 60 sec.
};

#endif //#ifndef __VEHICLE_SMARTED_H__
