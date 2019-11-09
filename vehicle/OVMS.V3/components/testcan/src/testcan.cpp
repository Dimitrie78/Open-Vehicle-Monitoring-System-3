/*
;    Project:       Open Vehicle Monitor System
;    Date:          11th Sep 2019
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011       Sonny Chen @ EPRO/DX
;    (C) 2018       Marcos Mezo
;    (C) 2019       Thomas Heuer @Dimitrie78
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
static const char *TAG = "testcan";

#include <fstream>
#include <sdkconfig.h>
#include <stdio.h>
#include "metrics_standard.h"


#include "testcan.h"

testcan MyTestCan __attribute__ ((init_priority (8500)));

testcan::testcan() {
  ESP_LOGI(TAG, "Initialising TESTCAN (8500)");

  MyCommandApp.RegisterCommand("testcan", "test", readfiletocan,"<rx/tx><path>",2,2);
}

testcan::~testcan() {
}

void testcan::ReadLogFileToCan( std::string path, bool send ) {
  FILE *sf = NULL;
  char s[256];
  char time[40];
  char canName[10];
  char canid[10];
  char candata[8][10];
  CAN_frame_t frame;
  
	sf = fopen(path.c_str(), "r"); // 142.080 1R11 5D7 00 00 00 31 57 00 C0
	if (!sf) {
		return;
	}
  canbus* sbus1 = (canbus*)MyPcpApp.FindDeviceByName("can1");
  canbus* sbus2 = (canbus*)MyPcpApp.FindDeviceByName("can2");
  while( fgets( s, 256, sf) != NULL ){
    memset(&candata, 0, sizeof(candata));
    sscanf( s, "%s %s %s %s %s %s %s %s %s %s %s", time, canName, canid, candata[0], candata[1], candata[2], candata[3], candata[4], candata[5], candata[6], candata[7]);
    if (strcmp(canName,"1R11")==0) {
      //ESP_LOGI(TAG, "%s %s %s %s %s %s %s %s %s %s %s", time, canName, canid, candata[0], candata[1], candata[2], candata[3], candata[4], candata[5], candata[6], candata[7]);
			
      memset(&frame, 0, sizeof(frame));
      int i=1;
      frame.origin = sbus1;
      frame.FIR.U = 0;
      frame.FIR.B.FF = CAN_frame_std;
      frame.MsgID = (int)strtol(canid,NULL,16);
      for(int k=0;k<7;k++){
        if(candata[k] == 0) break;
        else {
          frame.data.u8[k] = strtol(candata[k],NULL,16);
          i++;
        }
      }
      frame.FIR.B.DLC = i;
      
      if(send)
        sbus1->Write(&frame);
      else
        MyCan.IncomingFrame(&frame);
      //vTaskDelay(pdMS_TO_TICKS(100)); 
    }
    if (strcmp(canName,"2R11")==0) {
      memset(&frame, 0, sizeof(frame));
      int i=1;
      frame.origin = sbus2;
      frame.FIR.U = 0;
      frame.FIR.B.FF = CAN_frame_std;
      frame.MsgID = (int)strtol(canid,NULL,16);
      for(int k=0;k<7;k++){
        if(candata[k] == 0) break;
        else {
          frame.data.u8[k] = strtol(candata[k],NULL,16);
          i++;
        }
      }
      frame.FIR.B.DLC = i;
      
      if(send)
        sbus2->Write(&frame);
      else
        MyCan.IncomingFrame(&frame);
      //vTaskDelay(pdMS_TO_TICKS(100)); 
    }
  }

	fclose(sf);
  ESP_LOGI(TAG,"ReadLogFileToCan");
  return;
}

/**
 * readfiletocan
 */
void testcan::readfiletocan(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  std::string path;
  bool send;
  if (argv[1][0] == '/') {
    // A direct path specification
    path = std::string(argv[1]);
  }
  else {
    writer->puts("Error: argument must be '/sd/file.crtd'");
    return;
  }
  if (strcmp("tx", argv[0]) == 0) {
    send = true;
  } else if (strcmp("rx", argv[0]) == 0) {
    send = false;
  } else {
    writer->puts("Error: argument must be 'rx or tx'");
    return;
  }
  MyTestCan.ReadLogFileToCan( path, send );
  //writer->puts("OK");
}
