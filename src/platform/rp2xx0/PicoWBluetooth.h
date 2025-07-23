#pragma once

#include "BluetoothCommon.h"
#include <Arduino.h>

#include "hci.h"
#include "PhoneAPI.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "ble/att_db_util.h"
#include "ble/le_device_db.h"
#include "ble/sm.h"

class PicoWBluetooth : BluetoothApi {
protected:
    ~PicoWBluetooth();

public:
    void setup();

    void shutdown();

    void clearBonds();

    bool isConnected();

    int getRssi();

    void updateBatteryLevel(uint8_t uint8);
};
