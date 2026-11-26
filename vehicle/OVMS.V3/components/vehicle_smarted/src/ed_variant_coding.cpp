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

#include <stdio.h>
#include <string>
#include <iomanip>
#include "pcp.h"
#include "ovms_events.h"
#include "metrics_standard.h"
#include "ovms_notify.h"
#include "ovms_peripherals.h"

#include "vehicle_smarted.h"

void OvmsVehicleSmartED::VCInit() {
  variantCoding_cepc = false;
  variantCoding_ki = false;
  
  // init commands:
  OvmsCommand* vc;
  OvmsCommand* cepc;
  OvmsCommand* ki;
  vc = cmd_xse->RegisterCommand("vc", "Variant Coding");
  cepc = vc->RegisterCommand("cepc", "EV-Motor Steuergerät");
  cepc->RegisterCommand("start", "Start Variant Coding", shell_variant_coding_start);
  cepc->RegisterCommand("stop", "Stop Variant Coding", shell_variant_coding_stop);
  cepc->RegisterCommand("get", "get seed", shell_variant_coding_get_seed);
  cepc->RegisterCommand("send", "send key", shell_variant_coding_send_key, "<key...>", 4, 4);
  cepc->RegisterCommand("data", "get data", shell_variant_coding_get_data);
  cepc->RegisterCommand("code", "set data", shell_variant_coding_set_data, "<wippen> <on/off>", 2, 2);
  cepc->RegisterCommand("reset", "ECU reset", shell_variant_coding_ecu_reset);
  ki = vc->RegisterCommand("ki", "Kombi Instrument Steuergerät");
  ki->RegisterCommand("start", "Start Variant Coding", shell_variant_coding_start);
  ki->RegisterCommand("stop", "Stop Variant Coding", shell_variant_coding_stop);
  ki->RegisterCommand("get", "get seed", shell_variant_coding_get_seed);
  ki->RegisterCommand("send", "send key", shell_variant_coding_send_key, "<key...>", 4, 4);
  ki->RegisterCommand("data", "get data", shell_variant_coding_get_data);
  ki->RegisterCommand("code", "set data", shell_variant_coding_set_data);
  ki->RegisterCommand("reset", "ECU reset", shell_variant_coding_ecu_reset);
}

void OvmsVehicleSmartED::VCticker1() {
  if ((m_ticker % 3) == 0) {
    if (variantCoding_cepc && mt_bus_awake->AsBool())
      sendWake_VC_cepc();
    if (variantCoding_ki && mt_bus_awake->AsBool())
      sendWake_VC_ki();
  }
}

void OvmsVehicleSmartED::shell_variant_coding_start(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;
  
  string device = cmd->GetParent()->GetName();
  if (device == "cepc")
    smart->VC_cepc_start(verbosity, writer);
  if (device == "ki")
    smart->VC_ki_start(verbosity, writer);
  writer->puts("OK");
}

void OvmsVehicleSmartED::shell_variant_coding_stop(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;
  
  smart->VC_stop(verbosity, writer);
  writer->puts("OK");
}

void OvmsVehicleSmartED::shell_variant_coding_get_seed(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;
  
  string device = cmd->GetParent()->GetName();
  if (device == "cepc")
    smart->VC_cepc_get_seed(verbosity, writer);
  if (device == "ki")
    smart->VC_ki_get_seed(verbosity, writer);
}

void OvmsVehicleSmartED::shell_variant_coding_send_key(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;
  
  uint16_t txid = 0;
  uint8_t data[8] = {0x06, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  string device = cmd->GetParent()->GetName();
  if (device == "cepc") {
    txid = 0x7E5;
    data[2] = 0x0C;
  }
  if (device == "ki") {
    txid = 0x782;
    data[2] = 0x02;
  }
  
  for(int k=0;k<(argc);k++) {
    data[k+3] = strtol(argv[k],NULL,16);
  }
  
  smart->VC_send_key(txid, data, verbosity, writer);
}

void OvmsVehicleSmartED::shell_variant_coding_get_data(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;

  string device = cmd->GetParent()->GetName();
  if (device == "cepc")
    smart->VC_cepc_get_data(verbosity, writer);
  if (device == "ki")
    smart->VC_ki_get_data(verbosity, writer);
}

void OvmsVehicleSmartED::shell_variant_coding_set_data(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;

  bool wippen = false;
	if (strcmp("wippen", argv[0]) == 0) {
    if (strcmp("on", argv[1]) == 0)
      wippen = true;
    else if (strcmp("off", argv[1]) == 0)
      wippen = false;
  }

  string device = cmd->GetParent()->GetName();
  if (device == "cepc")
    smart->VC_cepc_send_data(wippen, verbosity, writer);
  if (device == "ki")
    smart->VC_ki_send_data(verbosity, writer);
}

void OvmsVehicleSmartED::shell_variant_coding_ecu_reset(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;

  string device = cmd->GetParent()->GetName();
  if (device == "cepc")
    smart->VC_ecu_reset(0x7E5);
  if (device == "ki")
    smart->VC_ecu_reset(0x782);
  
  writer->puts("ECU Reset OK");
}

void OvmsVehicleSmartED::VC_cepc_get_data(int verbosity, OvmsWriter* writer) {
  uint16_t txid = 0, rxid = 0;
  string request, response;
  
  txid = 0x7E5;
  rxid = 0x7ED;
  
  request = hexdecode("221001");
  
  // execute request:
  int err = ObdRequest(txid, rxid, request, response);
  if (err == -1) {
    writer->puts("ERROR: timeout waiting for response");
    return;
  } else if (err) {
    writer->printf("ERROR: request failed with response error code %02X\n", err);
    return;
  }
  
  if (response[16]&0x04)
    writer->puts("Recu Wippen aktiv");
  else
    writer->puts("Recu Wippen inaktiv");

  // output response as hex dump:
  writer->puts("Response:");
  char *buf = NULL;
  size_t rlen = response.size(), offset = 0;
  do {
    rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
    offset += 16;
    writer->puts(buf ? buf : "-");
  } while (rlen);
  if (buf)
    free(buf);
}

void OvmsVehicleSmartED::VC_cepc_start(int verbosity, OvmsWriter* writer) {
  uint8_t data[8] = {0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  m_enable_write = false;
  PollSetState(0);
  
  canbus *obd;
  obd = m_can1;
  
  obd->WriteStandard(0x7E5, 8, data);
  variantCoding_cepc = true;
}

void OvmsVehicleSmartED::VC_stop(int verbosity, OvmsWriter* writer) {
  variantCoding_cepc = false;
  variantCoding_ki = false;
  m_enable_write = true;
}

void OvmsVehicleSmartED::VC_cepc_get_seed(int verbosity, OvmsWriter* writer) {
  uint16_t txid = 0, rxid = 0;
  string request, response;
  
  txid = 0x7E5;
  rxid = 0x7ED;
  
  request = hexdecode("270B");
  
  // execute request:
  int err = ObdRequest(txid, rxid, request, response);
  if (err == -1) {
    writer->puts("ERROR: timeout waiting for response");
    return;
  } else if (err) {
    writer->printf("ERROR: request failed with response error code %02X\n", err);
    return;
  }
  
  // output response as hex dump:
  writer->puts("Response:");
  char *buf = NULL;
  size_t rlen = response.size(), offset = 0;
  do {
    rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
    offset += 16;
    writer->puts(buf ? buf : "-");
  } while (rlen);
  if (buf)
    free(buf);
}

void OvmsVehicleSmartED::VC_cepc_send_data(bool wippen, int verbosity, OvmsWriter* writer) {
  uint16_t txid = 0, rxid = 0;
  string request, response;
  bool coding = false;
  
  txid = 0x7E5;
  rxid = 0x7ED;
  
  request = hexdecode("221001");
  
  // execute request:
  int err = ObdRequest(txid, rxid, request, response);
  if (err == -1) {
    writer->puts("ERROR: timeout waiting for response");
    return;
  } else if (err) {
    writer->printf("ERROR: request failed with response error code %02X\n", err);
    return;
  }
  
  if (response[16]&0x04) {
    writer->puts("Recu Wippen aktiv");
    if (!wippen) {
      response[16] = response[16]-0x04;
      coding = true;
      writer->puts("Start Coding Recu Wippen off");
    }
  } else {
    writer->puts("Recu Wippen inaktiv");
    if (wippen) {
      response[16] = response[16]+0x04;
      coding = true;
      writer->puts("Start Coding Recu Wippen on");
    }
  }

  if (coding) {
    canbus *obd;
    obd = m_can1;

    uint8_t data[8] = {0x10, 0x3E, 0x2E, 0x10, 0x01, 0x00, 0x00, 0x00}; //10,3E,2E,10,01,18,87,40,
    data[5] = response[0];
    data[6] = response[1];
    data[7] = response[2];
    obd->WriteStandard(0x7E5, 8, data);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (flow_request) {
      data[0] = 0x21;
      data[1] = response[3];
      data[2] = response[4];
      data[3] = response[5];
      data[4] = response[6];
      data[5] = response[7];
      data[6] = response[8];
      data[7] = response[9];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x22;
      data[1] = response[10];
      data[2] = response[11];
      data[3] = response[12];
      data[4] = response[13];
      data[5] = response[14];
      data[6] = response[15];
      data[7] = response[16];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x23;
      data[1] = response[17];
      data[2] = response[18];
      data[3] = response[19];
      data[4] = response[20];
      data[5] = response[21];
      data[6] = response[22];
      data[7] = response[23];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x24;
      data[1] = response[24];
      data[2] = response[25];
      data[3] = response[26];
      data[4] = response[27];
      data[5] = response[28];
      data[6] = response[29];
      data[7] = response[30];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x25;
      data[1] = response[31];
      data[2] = response[32];
      data[3] = response[33];
      data[4] = response[34];
      data[5] = response[35];
      data[6] = response[36];
      data[7] = response[37];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x26;
      data[1] = response[38];
      data[2] = response[39];
      data[3] = response[40];
      data[4] = response[41];
      data[5] = response[42];
      data[6] = response[43];
      data[7] = response[44];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x27;
      data[1] = response[45];
      data[2] = response[46];
      data[3] = response[47];
      data[4] = response[48];
      data[5] = response[49];
      data[6] = response[50];
      data[7] = response[51];
      obd->WriteStandard(0x7E5, 8, data);
      vTaskDelay(pdMS_TO_TICKS(20));
      
      data[0] = 0x28;
      data[1] = response[52];
      data[2] = response[53];
      data[3] = response[54];
      data[4] = response[55];
      data[5] = response[56];
      data[6] = response[57];
      data[7] = response[58];
      obd->WriteStandard(0x7E5, 8, data);
      
      flow_request = false;
    }
  }
  // output response as hex dump:
  writer->puts("Response:");
  char *buf = NULL;
  size_t rlen = response.size(), offset = 0;
  do {
    rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
    offset += 16;
    writer->puts(buf ? buf : "-");
  } while (rlen);
  if (buf)
    free(buf);
}

void OvmsVehicleSmartED::VC_send_key(uint16_t txid, uint8_t *data, int verbosity, OvmsWriter* writer) {
  canbus *obd;
  obd = m_can1;
  
  obd->WriteStandard(txid, 8, data);
}

void OvmsVehicleSmartED::sendWake_VC_cepc() {
  uint8_t data[8] = {0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  canbus *obd;
  obd = m_can1;
  
  obd->WriteStandard(0x7E5, 8, data);
}

void OvmsVehicleSmartED::sendWake_VC_ki() {
  uint8_t data[8] = {0x02, 0x3E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  canbus *obd;
  obd = m_can1;
  
  obd->WriteStandard(0x782, 8, data);
}

void OvmsVehicleSmartED::VC_ki_start(int verbosity, OvmsWriter* writer) {
  uint8_t data[8] = {0x02, 0x10, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  m_enable_write = false;
  PollSetState(0);
  
  canbus *obd;
  obd = m_can1;
  
  obd->WriteStandard(0x782, 8, data);
  variantCoding_ki = true;
}

void OvmsVehicleSmartED::VC_ki_get_seed(int verbosity, OvmsWriter* writer) {
  uint16_t txid = 0, rxid = 0;
  string request, response;
  
  txid = 0x782;
  rxid = 0x783;
  
  request = hexdecode("2701");
  
  // execute request:
  int err = ObdRequest(txid, rxid, request, response);
  if (err == -1) {
    writer->puts("ERROR: timeout waiting for response");
    return;
  } else if (err) {
    writer->printf("ERROR: request failed with response error code %02X\n", err);
    return;
  }
  
  // output response as hex dump:
  writer->puts("Response:");
  char *buf = NULL;
  size_t rlen = response.size(), offset = 0;
  do {
    rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
    offset += 16;
    writer->puts(buf ? buf : "-");
  } while (rlen);
  if (buf)
    free(buf);
}

void OvmsVehicleSmartED::VC_ki_get_data(int verbosity, OvmsWriter* writer) {
  uint16_t txid = 0, rxid = 0;
  string request, response;
  
  txid = 0x782;
  rxid = 0x783;
  
  request = hexdecode("2110");
  
  // execute request:
  int err = ObdRequest(txid, rxid, request, response);
  if (err == -1) {
    writer->puts("ERROR: timeout waiting for response");
    return;
  } else if (err) {
    writer->printf("ERROR: request failed with response error code %02X\n", err);
    return;
  }
  
  if (response[1]&0x08)
    writer->puts("Gurtwarnung aktiv");
  else
    writer->puts("Gurtwarnung inaktiv");
  
  if (response[6]&0x01)
    writer->puts("Digital Tacho km/h");
  else if (response[6]&0x02)
    writer->puts("Digital Tacho mph");
  else
    writer->puts("Digital Tacho off");
  
  // output response as hex dump:
  writer->puts("Response:");
  char *buf = NULL;
  size_t rlen = response.size(), offset = 0;
  do {
    rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
    offset += 16;
    writer->puts(buf ? buf : "-");
  } while (rlen);
  if (buf)
    free(buf);
}

void OvmsVehicleSmartED::VC_ki_send_data(int verbosity, OvmsWriter* writer) {
  uint16_t txid = 0, rxid = 0;
  string request, response;
  
  txid = 0x782;
  rxid = 0x783;
  
  request = hexdecode("2110");
  
  // execute request:
  int err = ObdRequest(txid, rxid, request, response);
  if (err == -1) {
    writer->puts("ERROR: timeout waiting for response");
    return;
  } else if (err) {
    writer->printf("ERROR: request failed with response error code %02X\n", err);
    return;
  }
  
  if (response[1]&0x08) {
    writer->puts("Gurtwarnung aktiv");
    response[1] = response[1]-0x08;
  }

  canbus *obd;
  obd = m_can1;

  uint8_t data[8] = {0x10, 0x09, 0x3B, 0x10, 0x00, 0x00, 0x00, 0x00}; //10 09 3b 10 02 f8 bf c4
  data[4] = response[0];
  data[5] = response[1];
  data[6] = response[2];
  data[7] = response[3];
  obd->WriteStandard(0x782, 8, data);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  if (flow_request) {
    data[0] = 0x21;
    data[1] = response[4];
    data[2] = response[5];
    data[3] = response[6];
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    obd->WriteStandard(0x782, 8, data);
    flow_request = false;
  }
  
  
  // output response as hex dump:
  writer->puts("Response:");
  char *buf = NULL;
  size_t rlen = response.size(), offset = 0;
  do {
    rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
    offset += 16;
    writer->puts(buf ? buf : "-");
  } while (rlen);
  if (buf)
    free(buf);
}

void OvmsVehicleSmartED::VC_ecu_reset(uint16_t type) {
  uint8_t data[8] = {0x02, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

  canbus *obd;
  obd = m_can1;

  obd->WriteStandard(type, 8, data);
}

void OvmsVehicleSmartED::VC_IncomingFrame(uint16_t type, uint8_t* data) {
  ESP_LOGD(TAG, "%03x 8 %02x %02x %02x %02x %02x %02x %02x %02x", type, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  switch(type) {
    case 0x7ED:
    {
      if (data[0] == 0x30) {
        flow_request = true;
      }
      if (data[0] == 0x02) {
        if (data[1] == 0x7F && data[2] == 0x27) {
          ESP_LOGE(TAG, "Seed key error: code %02x", data[3]);
          seed_valid = false;
        }
        if (data[1] == 0x67 && data[2] == 0x0C) {
          ESP_LOGI(TAG, "Seed key Valid");
          seed_valid = true;
        }
      } // 03 7f 2e 33 | 03 6e 10 01
      if (data[0] == 0x03) {
        if (data[1] == 0x7F) {
          if (data[2] == 0x2E) {
            ESP_LOGE(TAG, "Coding error: code %02x", data[3]);
          }
        }
        if (data[1] == 0x6E && data[2] == 0x10 && data[3] == 0x01) {
          ESP_LOGI(TAG, "Coding accepted");
        }
      }
      break;
    }
    case 0x783:
    {
      if (data[0] == 0x30) {
        flow_request = true;
      }
      if (data[0] == 0x03) {
        if (data[1] == 0x7F) {
          if (data[2] == 0x27) {
            ESP_LOGE(TAG, "Seed key error: code %02x", data[3]);
            seed_valid = false;
          }
          if (data[2] == 0x3B) {
            ESP_LOGE(TAG, "Coding error: code %02x", data[3]);
          }
        }
        if (data[1] == 0x67 && data[2] == 0x02) {
          ESP_LOGI(TAG, "Seed key Valid");
          seed_valid = true;
        }
      }
      break;
    }
  }
}