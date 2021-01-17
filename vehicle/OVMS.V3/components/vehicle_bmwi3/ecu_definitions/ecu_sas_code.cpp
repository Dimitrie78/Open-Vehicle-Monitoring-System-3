
//
// Warning: don't edit - generated by generate_ecu_code.pl processing ../dev/sas_i1.json: SAS 22: Optional equipment system
// This generated code  makes it easier to process CANBUS messages from the SAS ecu in a BMW i3
//

  case I3_PID_SAS_LERNDATEN_RUECKSETZEN: {                                        // 0xABC9
    // ==========  Add your processing here ==========
    hexdump(rxbuf, type, pid);

    break;
  }

  case I3_PID_SAS_VDC0_LESEN: {                                                   // 0xD817
    if (datalen < 29) {
        ESP_LOGW(TAG, "Received %d bytes for %s, expected %d", datalen, "I3_PID_SAS_VDC0_LESEN", 29);
        break;
    }

    unsigned short STAT_VDC_SOLLSTROM_VL_WERT = (RXBUF_UINT(0));
        // Target current of the VDC channel in the front left / Sollstrom des VDC Kanals vorne links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_SOLLSTROM_VL_WERT", STAT_VDC_SOLLSTROM_VL_WERT, "\"mA\"");

    unsigned short STAT_VDC_SOLLSTROM_VR_WERT = (RXBUF_UINT(2));
        // Target current of the VDC channel in the front right / Sollstrom des VDC Kanals vorne rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_SOLLSTROM_VR_WERT", STAT_VDC_SOLLSTROM_VR_WERT, "\"mA\"");

    unsigned short STAT_VDC_SOLLSTROM_HL_WERT = (RXBUF_UINT(4));
        // Target current of the VDC channel at the rear left / Sollstrom des VDC Kanals hinten links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_SOLLSTROM_HL_WERT", STAT_VDC_SOLLSTROM_HL_WERT, "\"mA\"");

    unsigned short STAT_VDC_SOLLSTROM_HR_WERT = (RXBUF_UINT(6));
        // Set current of the VDC channel at the rear right / Sollstrom des VDC Kanals hinten rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_SOLLSTROM_HR_WERT", STAT_VDC_SOLLSTROM_HR_WERT, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_VL_WERT = (RXBUF_UINT(8));
        // Actual current of the VDC channel front left / Iststrom des VDC Kanals vorne links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_ISTSTROM_VL_WERT", STAT_VDC_ISTSTROM_VL_WERT, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_VR_WERT = (RXBUF_UINT(10));
        // Actual current of the VDC channel in the front right / Iststrom des VDC Kanals vorne rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_ISTSTROM_VR_WERT", STAT_VDC_ISTSTROM_VR_WERT, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_HL_WERT = (RXBUF_UINT(12));
        // Actual current of the VDC channel at the rear left / Iststrom des VDC Kanals hinten links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_ISTSTROM_HL_WERT", STAT_VDC_ISTSTROM_HL_WERT, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_HR_WERT = (RXBUF_UINT(14));
        // Actual current of the VDC channel at the rear right / Iststrom des VDC Kanals hinten rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_ISTSTROM_HR_WERT", STAT_VDC_ISTSTROM_HR_WERT, "\"mA\"");

    unsigned char STAT_VDC_STATUS_VL = (RXBUF_UCHAR(16));
        // Status of the front left VDC channel / Status des VDC Kanals vorne links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_STATUS_VL", STAT_VDC_STATUS_VL, "\"0-n\"");

    unsigned char STAT_VDC_STATUS_VR = (RXBUF_UCHAR(17));
        // Status of the VDC channel in the front right / Status des VDC Kanals vorne rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_STATUS_VR", STAT_VDC_STATUS_VR, "\"0-n\"");

    unsigned char STAT_VDC_STATUS_HL = (RXBUF_UCHAR(18));
        // Status of the VDC channel in the back left / Status des VDC Kanals hinten links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_STATUS_HL", STAT_VDC_STATUS_HL, "\"0-n\"");

    unsigned char STAT_VDC_STATUS_HR = (RXBUF_UCHAR(19));
        // Status of the VDC channel in the back right / Status des VDC Kanals hinten rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN", "STAT_VDC_STATUS_HR", STAT_VDC_STATUS_HR, "\"0-n\"");

    unsigned char STAT_KLEMMEN = (RXBUF_UCHAR(20));
        // Internal status of terminal KL15 0 = KL15 OFF 1 = KL15 ON / Interner Status der Klemme KL15 0 = KL15 AUS 1 =
        // KL15 AN
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN", "STAT_KLEMMEN", STAT_KLEMMEN, "\"0-n\"");

    float STAT_WHL_SPD_VL_WERT = (RXBUF_UINT(21)*0.0156f-511.984);
        // Front left wheel speed (from FlexRay) / Radgeschwindigkeit vorne links (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN", "STAT_WHL_SPD_VL_WERT", STAT_WHL_SPD_VL_WERT, "\"rad/s\"");

    float STAT_WHL_SPD_VR_WERT = (RXBUF_UINT(23)*0.0156f-511.984);
        // Wheel speed front right (from FlexRay) / Radgeschwindigkeit vorne rechts (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN", "STAT_WHL_SPD_VR_WERT", STAT_WHL_SPD_VR_WERT, "\"rad/s\"");

    float STAT_WHL_SPD_HL_WERT = (RXBUF_UINT(25)*0.0156f-511.984);
        // Rear left wheel speed (from FlexRay) / Radgeschwindigkeit hinten links (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN", "STAT_WHL_SPD_HL_WERT", STAT_WHL_SPD_HL_WERT, "\"rad/s\"");

    float STAT_WHL_SPD_HR_WERT = (RXBUF_UINT(27)*0.0156f-511.984);
        // Wheel speed rear right (from FlexRay) / Radgeschwindigkeit hinten rechts (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN", "STAT_WHL_SPD_HR_WERT", STAT_WHL_SPD_HR_WERT, "\"rad/s\"");

    // ==========  Add your processing here ==========
    hexdump(rxbuf, type, pid);

    break;
  }

  case I3_PID_SAS_VDC0_LESEN_0XD817: {                                            // 0xD817
    if (datalen < 29) {
        ESP_LOGW(TAG, "Received %d bytes for %s, expected %d", datalen, "I3_PID_SAS_VDC0_LESEN_0XD817", 29);
        break;
    }

    unsigned short STAT_VDC_SOLLSTROM_VL_WERT_0XD817 = (RXBUF_UINT(0));
        // Target current of the VDC channel in the front left / Sollstrom des VDC Kanals vorne links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_SOLLSTROM_VL_WERT_0XD817", STAT_VDC_SOLLSTROM_VL_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_SOLLSTROM_VR_WERT_0XD817 = (RXBUF_UINT(2));
        // Target current of the VDC channel in the front right / Sollstrom des VDC Kanals vorne rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_SOLLSTROM_VR_WERT_0XD817", STAT_VDC_SOLLSTROM_VR_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_SOLLSTROM_HL_WERT_0XD817 = (RXBUF_UINT(4));
        // Target current of the VDC channel at the rear left / Sollstrom des VDC Kanals hinten links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_SOLLSTROM_HL_WERT_0XD817", STAT_VDC_SOLLSTROM_HL_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_SOLLSTROM_HR_WERT_0XD817 = (RXBUF_UINT(6));
        // Set current of the VDC channel at the rear right / Sollstrom des VDC Kanals hinten rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_SOLLSTROM_HR_WERT_0XD817", STAT_VDC_SOLLSTROM_HR_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_VL_WERT_0XD817 = (RXBUF_UINT(8));
        // Actual current of the VDC channel front left / Iststrom des VDC Kanals vorne links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_ISTSTROM_VL_WERT_0XD817", STAT_VDC_ISTSTROM_VL_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_VR_WERT_0XD817 = (RXBUF_UINT(10));
        // Actual current of the VDC channel in the front right / Iststrom des VDC Kanals vorne rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_ISTSTROM_VR_WERT_0XD817", STAT_VDC_ISTSTROM_VR_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_HL_WERT_0XD817 = (RXBUF_UINT(12));
        // Actual current of the VDC channel at the rear left / Iststrom des VDC Kanals hinten links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_ISTSTROM_HL_WERT_0XD817", STAT_VDC_ISTSTROM_HL_WERT_0XD817, "\"mA\"");

    unsigned short STAT_VDC_ISTSTROM_HR_WERT_0XD817 = (RXBUF_UINT(14));
        // Actual current of the VDC channel at the rear right / Iststrom des VDC Kanals hinten rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_ISTSTROM_HR_WERT_0XD817", STAT_VDC_ISTSTROM_HR_WERT_0XD817, "\"mA\"");

    unsigned char STAT_VDC_STATUS_VL_0XD817 = (RXBUF_UCHAR(16));
        // Status of the front left VDC channel / Status des VDC Kanals vorne links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_STATUS_VL_0XD817", STAT_VDC_STATUS_VL_0XD817, "\"0-n\"");

    unsigned char STAT_VDC_STATUS_VR_0XD817 = (RXBUF_UCHAR(17));
        // Status of the VDC channel in the front right / Status des VDC Kanals vorne rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_STATUS_VR_0XD817", STAT_VDC_STATUS_VR_0XD817, "\"0-n\"");

    unsigned char STAT_VDC_STATUS_HL_0XD817 = (RXBUF_UCHAR(18));
        // Status of the VDC channel in the back left / Status des VDC Kanals hinten links
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_STATUS_HL_0XD817", STAT_VDC_STATUS_HL_0XD817, "\"0-n\"");

    unsigned char STAT_VDC_STATUS_HR_0XD817 = (RXBUF_UCHAR(19));
        // Status of the VDC channel in the back right / Status des VDC Kanals hinten rechts
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_VDC_STATUS_HR_0XD817", STAT_VDC_STATUS_HR_0XD817, "\"0-n\"");

    unsigned char STAT_KLEMMEN_0XD817 = (RXBUF_UCHAR(20));
        // Internal status of terminal KL15 0 = KL15 OFF 1 = KL15 ON / Interner Status der Klemme KL15 0 = KL15 AUS 1 =
        // KL15 AN
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%x%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_KLEMMEN_0XD817", STAT_KLEMMEN_0XD817, "\"0-n\"");

    float STAT_WHL_SPD_VL_WERT_0XD817 = (RXBUF_UINT(21)*0.0156f-511.984);
        // Front left wheel speed (from FlexRay) / Radgeschwindigkeit vorne links (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_WHL_SPD_VL_WERT_0XD817", STAT_WHL_SPD_VL_WERT_0XD817, "\"rad/s\"");

    float STAT_WHL_SPD_VR_WERT_0XD817 = (RXBUF_UINT(23)*0.0156f-511.984);
        // Wheel speed front right (from FlexRay) / Radgeschwindigkeit vorne rechts (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_WHL_SPD_VR_WERT_0XD817", STAT_WHL_SPD_VR_WERT_0XD817, "\"rad/s\"");

    float STAT_WHL_SPD_HL_WERT_0XD817 = (RXBUF_UINT(25)*0.0156f-511.984);
        // Rear left wheel speed (from FlexRay) / Radgeschwindigkeit hinten links (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_WHL_SPD_HL_WERT_0XD817", STAT_WHL_SPD_HL_WERT_0XD817, "\"rad/s\"");

    float STAT_WHL_SPD_HR_WERT_0XD817 = (RXBUF_UINT(27)*0.0156f-511.984);
        // Wheel speed rear right (from FlexRay) / Radgeschwindigkeit hinten rechts (von FlexRay)
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%.4f%s\n", "SAS", "VDC0_LESEN_0XD817", "STAT_WHL_SPD_HR_WERT_0XD817", STAT_WHL_SPD_HR_WERT_0XD817, "\"rad/s\"");

    // ==========  Add your processing here ==========
    hexdump(rxbuf, type, pid);

    break;
  }

  case I3_PID_SAS_STATUS_SWC_VERSIONEN_LESEN_ANZAHL_DATENSAETZE: {                // 0xDD33
    if (datalen < 2) {
        ESP_LOGW(TAG, "Received %d bytes for %s, expected %d", datalen, "I3_PID_SAS_STATUS_SWC_VERSIONEN_LESEN_ANZAHL_DATENSAETZE", 2);
        break;
    }

    unsigned short STAT_INDEX_DATENSATZ_WERT = (RXBUF_UINT(0));
        // - / -
    ESP_LOGD(TAG, "From ECU %s, pid %s: got %s=%u%s\n", "SAS", "STATUS_SWC_VERSIONEN_LESEN_ANZAHL_DATENSAETZE", "STAT_INDEX_DATENSATZ_WERT", STAT_INDEX_DATENSATZ_WERT, "");

    // ==========  Add your processing here ==========
    hexdump(rxbuf, type, pid);

    break;
  }

  case I3_PID_SAS_READ_EXCEPTION_DATA: {                                          // 0x4001
    if (datalen < 50) {
        ESP_LOGW(TAG, "Received %d bytes for %s, expected %d", datalen, "I3_PID_SAS_READ_EXCEPTION_DATA", 50);
        break;
    }

    // ==========  Add your processing here ==========
    hexdump(rxbuf, type, pid);

    break;
  }

  case I3_PID_SAS_CLEAR_EXCEPTION_DATA: {                                         // 0xF000
    // ==========  Add your processing here ==========
    hexdump(rxbuf, type, pid);

    break;
  }
