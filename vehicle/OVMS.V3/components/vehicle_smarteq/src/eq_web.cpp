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

#include <sdkconfig.h>
#ifdef CONFIG_OVMS_COMP_WEBSERVER

#include <stdio.h>
#include <string>
#include "ovms_metrics.h"
#include "ovms_events.h"
#include "ovms_config.h"
#include "ovms_command.h"
#include "metrics_standard.h"
#include "ovms_notify.h"
#include "ovms_webserver.h"

#include "vehicle_smarteq.h"

using namespace std;

#define _attr(text) (c.encode_html(text).c_str())
#define _html(text) (c.encode_html(text).c_str())


/**
 * WebInit: register pages
 */
void OvmsVehicleSmartEQ::WebInit()
{
  // vehicle menu:
  MyWebServer.RegisterPage("/xsq/features", "Features", WebCfgFeatures, PageMenu_Vehicle, PageAuth_Cookie);
  MyWebServer.RegisterPage("/xsq/battery", "Battery config", WebCfgBattery, PageMenu_Vehicle, PageAuth_Cookie);
  MyWebServer.RegisterPage("/xsq/cellmon", "BMS cell monitor", OvmsWebServer::HandleBmsCellMonitor, PageMenu_Vehicle, PageAuth_Cookie);
}

/**
 * WebDeInit: deregister pages
 */
void OvmsVehicleSmartEQ::WebDeInit()
{
  MyWebServer.DeregisterPage("/xsq/features");
  MyWebServer.DeregisterPage("/xsq/battery");
  MyWebServer.DeregisterPage("/xsq/cellmon");
}

/**
 * WebCfgFeatures: configure general parameters (URL /xsq/config)
 */
void OvmsVehicleSmartEQ::WebCfgFeatures(PageEntry_t& p, PageContext_t& c)
{
  std::string error, info;
  bool canwrite, led, ios, resettrip, resettotal, bsact, bsdbt;
  std::string bstime;

  if (c.method == "POST") {
    // process form submission:
    canwrite  = (c.getvar("canwrite") == "yes");
    led  = (c.getvar("led") == "yes");
    ios  = (c.getvar("ios") == "yes");
    resettrip  = (c.getvar("resettrip") == "yes");
    resettotal  =  (c.getvar("resettotal") == "yes");
    bsact  =  (c.getvar("bsact") == "yes");
    bsdbt  =  (c.getvar("bsdbt") == "yes");
    bstime  =  (c.getvar("bstime"));
    int bsactint = 0;
    int bsdbtint = -1;
    if(bsact){bsactint = 1;}else{bsactint = 2;};
    if(bsdbt){bsdbtint = 1;}else{bsdbtint = 0;};
    int bstimeint = atoi(bstime.c_str());
    char buf[10];
    if(bstimeint > 959){
      sprintf(buf, "1,%d,0,%d,-1,-1,%d",bsactint , bstimeint, bsdbtint);
    } else if((bstimeint < 960)&&(bstimeint > 59)){
      sprintf(buf, "1,%d,0,0%d,-1,-1,%d",bsactint , bstimeint, bsdbtint);
    } else if((bstimeint < 60)&&(bstimeint > 9)){
      sprintf(buf, "1,%d,0,00%d,-1,-1,%d",bsactint , bstimeint, bsdbtint);
    } else {
      sprintf(buf, "1,%d,0,000%d,-1,-1,%d",bsactint , bstimeint, bsdbtint);
    }
      

    if (error == "") {
      // success:
      MyConfig.SetParamValueBool("xsq", "canwrite", canwrite);
      MyConfig.SetParamValueBool("xsq", "led", led);
      MyConfig.SetParamValueBool("xsq", "ios_tpms_fix", ios);
      MyConfig.SetParamValueBool("xsq", "resettrip", resettrip);
      MyConfig.SetParamValueBool("xsq", "resettotal", resettotal);
      MyConfig.SetParamValue("usr", "b.data", string(buf));

      info = "<p class=\"lead\">Success!</p><ul class=\"infolist\">" + info + "</ul>";
      c.head(200);
      c.alert("success", info.c_str());
      MyWebServer.OutputHome(p, c);
      c.done();
      return;
    }

    // output error, return to form:
    error = "<p class=\"lead\">Error!</p><ul class=\"errorlist\">" + error + "</ul>";
    c.head(400);
    c.alert("danger", error.c_str());
  }
  else {
    // read configuration:
    canwrite  = MyConfig.GetParamValueBool("xsq", "canwrite", false);
    led  = MyConfig.GetParamValueBool("xsq", "led", false);
    ios  = MyConfig.GetParamValueBool("xsq", "ios_tpms_fix", false);
    resettrip  = MyConfig.GetParamValueBool("xsq", "resettrip", false);
    resettotal  = MyConfig.GetParamValueBool("xsq", "resettotal", false);
    bsact  =  MyConfig.GetParamValueBool("usr", "b.activated", false);
    bsdbt  =  MyConfig.GetParamValueBool("usr", "b.scheduled_2", false);
    bstime  =  MyConfig.GetParamValue("usr", "b.scheduled", "0515");
    c.head(200);
  }

  // generate form:
  c.panel_start("primary", "smart EQ feature configuration");
  c.form_start(p.uri);
  
  c.input_checkbox("Enable CAN write(Poll)", "canwrite", canwrite,
    "<p>Controls overall CAN write access, some functions depend on this.</p>");
  c.input_checkbox("Enable/Disable Online state LED when installed", "led", led,
    "<p>RED=Internet no, BLUE=Internet yes, GREEN=Server v2 connected.<br>EGPIO Port 7,8,9 are used</p>");
  c.input_checkbox("Enable IOS TPMS fix", "ios", ios,
    "<p>Set External Temp to TPMS Temps to Display Tire Pressurs in IOS</p>");
  c.fieldset_start("Trip (Car site) and long term kWh/100km");
  // trip reset
  c.input_checkbox("Enable Reset Trip when Charging", "resettrip", resettrip,
    "<p>Enable = Reset Trip Values when Chaging, Disable = Reset Trip Values when Driving</p>");
  c.input_checkbox("Enable Reset kWh/100km when Car switched on", "resettotal", resettotal,
    "<p>Enable = Reset kWh/100km Values when Car switched on, auto Disabled when reseted</p>");
  // scheduled_booster plugin
  c.fieldset_start("Climate/Heater start timer");
  c.input_checkbox("Enable Climate/Heater at time", "bsact", bsact,
    "<p>Enable = start Climate/Heater at time</p>");
  c.input_checkbox("Enable two time activation Climate/Heater", "bsdbt", bsdbt,
    "<p>Enable = this option adds a 2nd Climate/Heater start </p>");
  c.input_text("time: ", "bstime", bstime.c_str(), "0515","<p>Time: 5:15 = 515 or 15:30 = 1530</p>");

  c.print("<hr>");
  c.input_button("default", "Save");
  c.form_end();
  c.panel_end();

  c.done();
}

/**
 * WebCfgBattery: configure battery parameters (URL /xnl/battery)
 */
void OvmsVehicleSmartEQ::WebCfgBattery(PageEntry_t& p, PageContext_t& c)
{
  std::string error;
  //  suffsoc          	Sufficient SOC [%] (Default: 0=disabled)
  //  suffrange        	Sufficient range [km] (Default: 0=disabled)
  std::string suffrange, suffsoc, cell_interval_drv, cell_interval_chg;

  if (c.method == "POST") {
    // process form submission:
    suffrange = c.getvar("suffrange");
    suffsoc   = c.getvar("suffsoc");
    cell_interval_drv = c.getvar("cell_interval_drv");
    cell_interval_chg = c.getvar("cell_interval_chg");

    // check:
    if (!suffrange.empty()) {
      float n = atof(suffrange.c_str());
      if (n < 0)
        error += "<li data-input=\"suffrange\">Sufficient range invalid, must be &ge; 0</li>";
    }
    if (!suffsoc.empty()) {
      float n = atof(suffsoc.c_str());
      if (n < 0 || n > 100)
        error += "<li data-input=\"suffsoc\">Sufficient SOC invalid, must be 0…100</li>";
    }

    if (error == "") {
      // store:
      MyConfig.SetParamValue("xsq", "suffrange", suffrange);
      MyConfig.SetParamValue("xsq", "suffsoc", suffsoc);
      MyConfig.SetParamValue("xsq", "cell_interval_drv", cell_interval_drv);
      MyConfig.SetParamValue("xsq", "cell_interval_chg", cell_interval_chg);

      c.head(200);
      c.alert("success", "<p class=\"lead\">SmartED3 battery setup saved.</p>");
      MyWebServer.OutputHome(p, c);
      c.done();
      return;
    }

    // output error, return to form:
    error = "<p class=\"lead\">Error!</p><ul class=\"errorlist\">" + error + "</ul>";
    c.head(400);
    c.alert("danger", error.c_str());
  }
  else {
    // read configuration:
    suffrange = MyConfig.GetParamValue("xsq", "suffrange", "0");
    suffsoc   = MyConfig.GetParamValue("xsq", "suffsoc", "0");
    cell_interval_drv = MyConfig.GetParamValue("xsq", "cell_interval_drv", "60");
    cell_interval_chg = MyConfig.GetParamValue("xsq", "cell_interval_chg", "60");

    c.head(200);
  }

  // generate form:

  c.panel_start("primary", "Smart ED Battery Setup");
  c.form_start(p.uri);

  c.fieldset_start("Charge control");

  c.input_slider("Sufficient range", "suffrange", 3, "km",
    atof(suffrange.c_str()) > 0, atof(suffrange.c_str()), 75, 0, 150, 1,
    "<p>Default 0=off. Notify/stop charge when reaching this level.</p>");

  c.input_slider("Sufficient SOC", "suffsoc", 3, "%",
    atof(suffsoc.c_str()) > 0, atof(suffsoc.c_str()), 80, 0, 100, 1,
    "<p>Default 0=off. Notify/stop charge when reaching this level.</p>");

  c.fieldset_end();
  
  c.fieldset_start("BMS Cell Monitoring");
  c.input_slider("Update interval driving", "cell_interval_drv", 3, "s",
    atof(cell_interval_drv.c_str()) > 0, atof(cell_interval_drv.c_str()),
    60, 0, 300, 1,
    "<p>Default 60 seconds, 0=off.</p>");
  c.input_slider("Update interval charging", "cell_interval_chg", 3, "s",
    atof(cell_interval_chg.c_str()) > 0, atof(cell_interval_chg.c_str()),
    60, 0, 300, 1,
    "<p>Default 60 seconds, 0=off.</p>");
  
  c.fieldset_end();

  c.print("<hr>");
  c.input_button("default", "Save");
  c.form_end();
  c.panel_end();
  c.done();
}

/**
 * GetDashboardConfig: smart ED specific dashboard setup
 */
void OvmsVehicleSmartEQ::GetDashboardConfig(DashboardConfig& cfg)
{
  // Speed:
  dash_gauge_t speed_dash(NULL,Kph);
  speed_dash.SetMinMax(0, 145, 5);
  speed_dash.AddBand("green", 0, 70);
  speed_dash.AddBand("yellow", 70, 100);
  speed_dash.AddBand("red", 100, 145);

  // Voltage:
  dash_gauge_t voltage_dash(NULL,Volts);
  voltage_dash.SetMinMax(260, 400);
  voltage_dash.AddBand("red", 260, 305);
  voltage_dash.AddBand("yellow", 305, 355);
  voltage_dash.AddBand("green", 355, 400);

  // SOC:
  dash_gauge_t soc_dash("SOC ",Percentage);
  soc_dash.SetMinMax(0, 100);
  soc_dash.AddBand("red", 0, 12.5);
  soc_dash.AddBand("yellow", 12.5, 25);
  soc_dash.AddBand("green", 25, 100);

  // Efficiency:
  dash_gauge_t eff_dash(NULL,WattHoursPK);
  eff_dash.SetMinMax(0, 300);
  eff_dash.AddBand("green", 0, 150);
  eff_dash.AddBand("yellow", 150, 250);
  eff_dash.AddBand("red", 250, 300);

  // Power:
  dash_gauge_t power_dash(NULL,kW);
  power_dash.SetMinMax(-30, 30);
  power_dash.AddBand("violet", -30, 0);
  power_dash.AddBand("green", 0, 15);
  power_dash.AddBand("yellow", 15, 25);
  power_dash.AddBand("red", 25, 30);

  // Charger temperature:
  dash_gauge_t charget_dash("CHG ",Celcius);
  charget_dash.SetMinMax(20, 80);
  charget_dash.SetTick(20);
  charget_dash.AddBand("normal", 20, 65);
  charget_dash.AddBand("red", 65, 80);

  // Battery temperature:
  dash_gauge_t batteryt_dash("BAT ",Celcius);
  batteryt_dash.SetMinMax(-15, 65);
  batteryt_dash.SetTick(25);
  batteryt_dash.AddBand("red", -15, 0);
  batteryt_dash.AddBand("normal", 0, 50);
  batteryt_dash.AddBand("red", 50, 65);

  // Inverter temperature:
  dash_gauge_t invertert_dash("PEM ",Celcius);
  invertert_dash.SetMinMax(20, 80);
  invertert_dash.SetTick(20);
  invertert_dash.AddBand("normal", 20, 70);
  invertert_dash.AddBand("red", 70, 80);

  // Motor temperature:
  dash_gauge_t motort_dash("MOT ",Celcius);
  motort_dash.SetMinMax(50, 125);
  motort_dash.SetTick(25);
  motort_dash.AddBand("normal", 50, 110);
  motort_dash.AddBand("red", 110, 125);

  std::ostringstream str;
  str << "yAxis: ["
      << speed_dash << "," // Speed
      << voltage_dash << "," // Voltage
      << soc_dash << "," // SOC
      << eff_dash << "," // Efficiency
      << power_dash << "," // Power
      << charget_dash << "," // Charger temperature
      << batteryt_dash << "," // Battery temperature
      << invertert_dash << "," // Inverter temperature
      << motort_dash // Motor temperature
      << "]";
  cfg.gaugeset1 = str.str();
}

#endif //CONFIG_OVMS_COMP_WEBSERVER
