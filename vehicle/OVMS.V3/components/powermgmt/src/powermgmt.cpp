/*
;    Project:       Open Vehicle Monitor System
;    Module:        Power Management Webserver
;    Date:          3rd September 2019
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2019        Marko Juhanne
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
*/

#include "ovms_log.h"
static const char *TAG = "powermgmt";

#include "ovms_notify.h"
#include "ovms_events.h"
#include "ovms_config.h"
#include "powermgmt.h"
#include "ovms_peripherals.h"
#include "metrics_standard.h"
#include "esp_sleep.h"

powermgmt MyPowerMgmt __attribute__ ((init_priority (8500)));

powermgmt::powermgmt()
  {
  ESP_LOGI(TAG, "Initialising POWERMGMT (8500)");

  using std::placeholders::_1;
  using std::placeholders::_2;
  MyEvents.RegisterEvent(TAG, "ticker.1", std::bind(&powermgmt::Ticker1, this, _1, _2));
  MyEvents.RegisterEvent(TAG, "config.changed", std::bind(&powermgmt::ConfigChanged, this, _1, _2));
  MyEvents.RegisterEvent(TAG, "config.mounted", std::bind(&powermgmt::ConfigChanged, this, _1, _2));

  m_enabled = false;
  m_notcharging_timer = 0;
  m_12v_alert_timer = 0;
  m_charging = false;
  m_modem_off = false;
  m_wifi_off = false;

  MyConfig.RegisterParam("power", "Power management", true, true);
  ConfigChanged("config.mounted", NULL);

#ifdef CONFIG_OVMS_COMP_WEBSERVER
  WebInit();
#endif  
  }

powermgmt::~powermgmt()
  {
#ifdef CONFIG_OVMS_COMP_WEBSERVER
  WebCleanup();
#endif  
  }

void powermgmt::ConfigChanged(std::string event, void* data)
  {
  OvmsConfigParam* param = (OvmsConfigParam*) data;

  // read vehicle framework config:
  if (!param || param->GetName() == "power")
    {
    m_enabled = MyConfig.GetParamValueBool("power", "enabled", false);
    m_modemoff_delay = MyConfig.GetParamValueInt("power", "modemoff_delay", POWERMGMT_MODEMOFF_DELAY);
    m_wifioff_delay = MyConfig.GetParamValueInt("power", "wifioff_delay", POWERMGMT_WIFIOFF_DELAY);
    m_12v_shutdown_delay = MyConfig.GetParamValueInt("power", "12v_shutdown_delay", POWERMGMT_12V_SHUTDOWN_DELAY);
    }
  if (!param || param->GetName() == "vehicle")
    {
    StandardMetrics.ms_v_bat_12v_voltage_ref->SetValue(MyConfig.GetParamValueFloat("vehicle", "12v.ref", 12.6));
    }
  }

void powermgmt::Ticker1(std::string event, void* data)
  {
  if (!m_charging)
    m_notcharging_timer++;
  
  if (StandardMetrics.ms_v_env_charging12v->AsBool())
    {
    if (!m_charging)
      {
      ESP_LOGI(TAG,"Charging 12V battery..");
      m_charging=true;
      // Turn on WiFi and modem if they were previously turned off by us
#ifdef CONFIG_OVMS_COMP_WIFI
      if (m_wifi_off)
        {
        MyPeripherals->m_esp32wifi->SetPowerMode(On);
        m_wifi_off = false;
        }
#endif
#ifdef CONFIG_OVMS_COMP_MODEM_SIMCOM
      if (m_modem_off)
        {
        MyPeripherals->m_simcom->SetPowerMode(On);
        m_modem_off = false;
        }
#endif
      }
    } 
  else
    {
    if (m_charging)
      {
      ESP_LOGI(TAG,"No longer charging 12V battery..");
      m_charging=false;
      m_notcharging_timer=0;
      }
    }

  if (!m_enabled)
    return;

#ifdef CONFIG_OVMS_COMP_WIFI
  if (m_wifioff_delay && m_notcharging_timer > m_wifioff_delay*60*60) // hours to seconds
    {
    if (MyPeripherals->m_esp32wifi->GetPowerMode()==On)
      {
      ESP_LOGW(TAG,"Powering off wifi");
      MyNotify.NotifyString("alert", "power.wifi", "Powermgmt: Powering off wifi");
      MyEvents.SignalEvent("powermgmt.wifi.off",NULL);
      vTaskDelay(500 / portTICK_PERIOD_MS); // make sure all notifications all transmitted before powerring down WiFI 
      MyPeripherals->m_esp32wifi->SetPowerMode(Off);
      m_wifi_off = true;
      }
    }
#endif

#ifdef CONFIG_OVMS_COMP_MODEM_SIMCOM
  if (m_modemoff_delay && m_notcharging_timer > m_modemoff_delay*60*60) // hours to seconds
    {
    if (MyPeripherals->m_simcom->GetPowerMode()==On)
      {
      ESP_LOGW(TAG,"Powering off modem");
      MyNotify.NotifyString("alert", "power.modem", "Powermgmt: Powering off modem");
      MyEvents.SignalEvent("powermgmt.modem.off",NULL);
      vTaskDelay(500 / portTICK_PERIOD_MS); // make sure all notifications all transmitted before powerring down modem 
      MyPeripherals->m_simcom->SetPowerMode(Off);
      m_modem_off = true;
      }
    }
#endif

  if (StandardMetrics.ms_v_bat_12v_voltage_alert->AsBool())
    {
    m_12v_alert_timer++;
    if (m_12v_alert_timer > m_12v_shutdown_delay*60) // minutes to seconds
      {
      m_12v_alert_timer = 0;
      
      ESP_LOGE(TAG,"Ongoing 12V battery alert time limit exceeded! Shutting down OVMS..");
      MyNotify.NotifyString("alert", "power.12v.alert", "Powermgmt: Ongoing 12V battery alert time limit exceeded! Shutting down OVMS..");
      MyEvents.SignalEvent("powermgmt.ovms.shutdown",NULL);
      vTaskDelay(3000 / portTICK_PERIOD_MS); // make sure all notifications all transmitted before powerring down OVMS

#ifdef CONFIG_OVMS_COMP_MODEM_SIMCOM
      if (MyPeripherals->m_simcom->GetPowerMode()==On)
        {
        MyPeripherals->m_simcom->SetPowerMode(Off);
        m_modem_off = true;
        }
#endif
#ifdef CONFIG_OVMS_COMP_WIFI
      if (MyPeripherals->m_esp32wifi->GetPowerMode()==On)
        {
        MyPeripherals->m_esp32wifi->SetPowerMode(Off);
        m_wifi_off = true;
        }
#endif
#ifdef CONFIG_OVMS_COMP_EXTERNAL_SWCAN
      MyPeripherals->m_mcp2515_swcan->SetPowerMode(Off);
#endif
#ifdef CONFIG_OVMS_COMP_MCP2515
      MyPeripherals->m_mcp2515_1->SetPowerMode(Off);
      MyPeripherals->m_mcp2515_2->SetPowerMode(Off);
#endif
#ifdef CONFIG_OVMS_COMP_ESP32CAN
      MyPeripherals->m_esp32can->SetPowerMode(Off);
#endif
#ifdef CONFIG_OVMS_COMP_EXT12V
      MyPeripherals->m_ext12v->SetPowerMode(Off);
#endif

      vTaskDelay(1000 / portTICK_PERIOD_MS);
      
      esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
      if (cause != ESP_SLEEP_WAKEUP_ULP) {
          ESP_LOGI(TAG,"Not ULP wakeup");
          //init_ulp_program();
      } else {
          ESP_LOGI(TAG,"Deep sleep wakeup");
          ESP_LOGI(TAG,"ULP did %d measurements since last reset", ulp_sample_counter & UINT16_MAX);
          ESP_LOGI(TAG,"Thresholds:  low=%d  high=%d", ulp_low_thr, ulp_high_thr);
          ulp_last_result &= UINT16_MAX;
          ESP_LOGI(TAG,"Value=%d was %s threshold", ulp_last_result, ulp_last_result < ulp_low_thr ? "below" : "above");
      }
      init_ulp_program();
      ESP_LOGI(TAG,"Entering deep sleep\n");
      start_ulp_program();
      ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
      
      MyPeripherals->m_esp32->SetPowerMode(Off);
      }
    }
  else
    {
    m_12v_alert_timer = 0;
    }
  }

void powermgmt::init_ulp_program()
{
  esp_err_t err = ulp_load_binary(0, ulp_powermgmt_bin_start,
          (ulp_powermgmt_bin_end - ulp_powermgmt_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  /* Configure ADC channel */
  /* Note: when changing channel here, also change 'adc_channel' constant
     in adc.S */
  //adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  //adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_ulp_enable();

  /* Set low and high thresholds, approx. 1.35V - 1.75V*/
  float f = MyConfig.GetParamValueFloat("system.adc", "factor12v", 195.7);
  float v = StandardMetrics.ms_v_bat_12v_voltage_ref->AsFloat(0);
  int h = f*v;
  ulp_low_thr = 0;
  ulp_high_thr = h;

  /* Set ULP wake up period to 1000ms */
  ulp_set_wakeup_period(0, 1000000);

  /* Disconnect GPIO12 and GPIO15 to remove current drain through
   * pullup/pulldown resistors.
   * GPIO12 may be pulled high to select flash voltage.
   */
  rtc_gpio_isolate(GPIO_NUM_12);
  //rtc_gpio_isolate(GPIO_NUM_15); // SD Card
  esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}

void powermgmt::start_ulp_program()
{
  /* Reset sample counter */
  ulp_sample_counter = 0;

  /* Start the program */
  esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err);
}
