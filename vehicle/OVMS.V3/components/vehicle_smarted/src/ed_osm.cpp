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
 */

#include "ovms_log.h"
static const char *TAG = "v-smarted";

#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string>
#include <string.h>
#include "ovms_http.h"
#include "ovms_netmanager.h"
#include "ovms_location.h"

#include "vehicle_smarted.h"

int OvmsVehicleSmartED::GetOsmSpeed() {
  char lat[20];
  char lon[20];
  gcvt(MyLocations.m_latitude, 10, lat);
  gcvt(MyLocations.m_longitude, 10, lon);
  std::string speed = "";
  std::string url = "ovms.dimitrie.eu/osm";

  //std::string url = "ovms.dimitrie.eu/firmware/ota/v3.1/edge";
  url.append("/ovms3.php");
  url.append("?lat=");
  url.append(lat);
  url.append("&lon=");
  url.append(lon);
  
  if (MyNetManager.m_connected_any) {
    OvmsHttpClient http(url);
    if (http.IsOpen() && http.ResponseCode() == 200 && http.BodyHasLine()) {
      speed = http.BodyReadLine();
    }
    else speed = "1";
    http.Disconnect();
  } else speed = "2";
  ESP_LOGV(TAG, "Speed: %s", speed.c_str());
  return atoi(speed.c_str());
}

void xse_osm(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  if (MyVehicleFactory.m_currentvehicle==NULL) {
    writer->puts("Error: No vehicle module selected");
    return;
  }
  OvmsVehicleSmartED* smart = (OvmsVehicleSmartED*) MyVehicleFactory.ActiveVehicle();
  int speed = smart->GetOsmSpeed();
  writer->printf("Speed: %d \n", speed);
}
