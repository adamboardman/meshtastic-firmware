#include "PicoWBluetooth.h"

#include <pico/cyw43_arch.h>

#include "btstack.h"
#include "hci_dump_embedded_stdout.h"
#include "hci.h"
#include "main.h"
#include "MeshService.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "ble/att_db_util.h"
#include "ble/le_device_db.h"
#include "ble/sm.h"
#include "include/meshtastic-profile.h"

//we use a circular buffer to store logging as we are often in an interrupt service

typedef struct {
    hci_con_handle_t connection_handle;
    int notification_enabled;
    uint16_t value_handle;
    uint8_t *data;
    int data_size;
    bool connected;
} le_connection_t;

#define BLE_ADVERTISING_MAX_LENGTH 31
#define BLE_FLAG_LE_LIMITED_DISC_MODE (0x01) // LE Limited Discoverable Mode
#define BLE_FLAG_LE_GENERAL_DISC_MODE (0x02) // LE General Discoverable Mode
#define BLE_FLAG_BR_EDR_NOT_SUPPORTED (0x04) // BR/EDR not supported
#define BLE_FLAG_LE_BR_EDR_CONTROLLER (0x08) // Simultaneous LE and BR/EDR, Controller
#define BLE_FLAG_LE_BR_EDR_HOST (0x10) // Simultaneous LE and BR/EDR, Host
#define BLE_FLAGS_LE_ONLY_LIMITED_DISC_MODE (BLE_FLAG_LE_LIMITED_DISC_MODE | BLE_FLAG_BR_EDR_NOT_SUPPORTED)
#define BLE_FLAGS_LE_ONLY_GENERAL_DISC_MODE (BLE_FLAG_LE_GENERAL_DISC_MODE | BLE_FLAG_BR_EDR_NOT_SUPPORTED)

//an additional 31 bytes of data can be given in response to a scan
static uint8_t scan_response_meshtastic_data[] = {
    //0xXX (the length of the subsequent data, {d0(type), d01-dXX}
    //6ba1b218-15a8-461f-9fa8-5dcae273eafd - meshtastic
    17, BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0xfd, 0xea, 0x73, 0xe2, 0xca, 0x5d, 0xa8,
    0x9f, 0x1f, 0x46, 0xa8, 0x15, 0x18, 0xb2, 0xa1, 0x6b,
};
static const uint8_t scan_response_meshtastic_data_length = sizeof(scan_response_meshtastic_data);
static_assert(scan_response_meshtastic_data_length <= 31);

le_connection_t le_connection_temp;
le_connection_t le_connection_battery;
le_connection_t le_connection_fromnum;
le_connection_t *le_connection = nullptr;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;
static btstack_context_callback_registration_t notify_context_callback_registration;

uint8_t battery_level = 0;
bool connection_secured = false;
uint32_t from_num = 0;

void notify_callback_handler(void *context) {
    LOG_INFO("notify_callback_handler calling att_server_notify");
    if (context == &le_connection_fromnum) {
        const int err = att_server_notify(le_connection_fromnum.connection_handle,
                                          le_connection_fromnum.value_handle,
                                          le_connection_fromnum.data,
                                          le_connection_fromnum.data_size);
        if (err) {
            LOG_INFO("notify_callback_handler - error!");
        }
    }
    //TODO support other notify callbacks
}

class BluetoothPhoneAPI : public PhoneAPI {
    void onNowHasData(uint32_t fromRadioNum) override;

    bool checkIsConnected() override;
};

static BluetoothPhoneAPI *bluetoothPhoneAPI = nullptr;

/**
 * Subclasses can use this as a hook to provide custom notifications for their transport (i.e. bluetooth notifies)
 */
void BluetoothPhoneAPI::onNowHasData(uint32_t fromRadioNum) {
    // LOG_INFO("BLE notify fromNum %d", fromRadioNum);
    __lockBluetooth();

    from_num = fromRadioNum;
    if (le_connection_fromnum.notification_enabled) {
        // LOG_DEBUG("onNowHasData - att_server_request_to_send_notification");
        le_connection_fromnum.value_handle = ATT_CHARACTERISTIC_ed9da18c_a800_4f66_a670_aa7547e34453_01_VALUE_HANDLE;
        le_connection_fromnum.data = reinterpret_cast<uint8_t *>(&from_num);
        le_connection_fromnum.data_size = sizeof(from_num);
        notify_context_callback_registration.callback = &notify_callback_handler;
        notify_context_callback_registration.context = &le_connection_fromnum;
        att_server_request_to_send_notification(&notify_context_callback_registration, le_connection_fromnum.connection_handle);
    }
    __unlockBluetooth();
}

/// Check the current underlying physical link to see if the client is currently connected
bool BluetoothPhoneAPI::checkIsConnected() {
    // see also PicoWBluetooth::isConnected()
    return le_connection_fromnum.connected || le_connection_fromnum.notification_enabled || connection_secured;
}

static uint8_t fromRadioBytes[meshtastic_FromRadio_size];
static size_t fromRadioNumBytes = 0;
static uint8_t lastToRadio[MAX_TO_FROM_RADIO_SIZE];

char print_scratch[MAX_TO_FROM_RADIO_SIZE];

void print_named_data(const char *name, uint8_t *data, uint16_t data_size) {
    memset(print_scratch, 0, MAX_TO_FROM_RADIO_SIZE);
    std::sprintf(print_scratch, "%s:", name);
    for (int i = 0; i < data_size && strlen(print_scratch) < MAX_TO_FROM_RADIO_SIZE - 10; i++) {
        std::sprintf(&print_scratch[strlen(print_scratch)], "%02x", data[i]);
    }
    LOG_DEBUG("%s", print_scratch);

    memset(print_scratch, 0, MAX_TO_FROM_RADIO_SIZE);
    std::sprintf(print_scratch, "%s[c]:", name);
    for (int i = 0; i < data_size && strlen(print_scratch) < MAX_TO_FROM_RADIO_SIZE - 10; i++) {
        if (data[i] >= 0x20 && data[i] < 0x7e) {
            std::sprintf(&print_scratch[strlen(print_scratch)], "%c", data[i]);
        } else {
            print_scratch[strlen(print_scratch)] = ' ';
        }
    }
    LOG_DEBUG("%s", print_scratch);
}

static le_connection_t *connection_for_conn_handle(const hci_con_handle_t connection_handle, const uint16_t att_handle) {
    //assume single connection for now
    le_connection_t *le_connection = nullptr;
    switch (att_handle) {
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE:
            le_connection = &le_connection_temp;
            le_connection->value_handle = ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE;
            break;
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL_01_CLIENT_CONFIGURATION_HANDLE:
            le_connection = &le_connection_battery;
            le_connection->value_handle = ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL_01_VALUE_HANDLE;
            break;
        case ATT_CHARACTERISTIC_ed9da18c_a800_4f66_a670_aa7547e34453_01_CLIENT_CONFIGURATION_HANDLE:
            le_connection = &le_connection_fromnum;
            le_connection->value_handle = ATT_CHARACTERISTIC_ed9da18c_a800_4f66_a670_aa7547e34453_01_VALUE_HANDLE;
            break;
    }
    le_connection->connection_handle = connection_handle;
    le_connection->notification_enabled = false;
    le_connection->data = nullptr;
    le_connection->data_size = 0;
    return le_connection;
}

uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer,
                           uint16_t buffer_size) {
    UNUSED(connection_handle);
    // LOG_DEBUG("att_read_callback(%02x,%02x,%d,buffer,%d)", connection_handle, att_handle, offset, buffer_size);

    uint16_t size_used_or_needed = 0;
    switch (att_handle) {
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE:
            // poll_temp();
            // size_used_or_needed = att_read_callback_handle_little_endian_16(current_temp, offset, buffer, buffer_size);
            break;
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL_01_VALUE_HANDLE:
            // poll_battery_level();
            size_used_or_needed = att_read_callback_handle_little_endian_16(battery_level, offset, buffer, buffer_size);
            break;
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE:
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL_01_CLIENT_CONFIGURATION_HANDLE:
        case ATT_CHARACTERISTIC_ed9da18c_a800_4f66_a670_aa7547e34453_01_CLIENT_CONFIGURATION_HANDLE:
        case ATT_CHARACTERISTIC_5a3d6e49_06e6_4423_9944_e9de8cdf9547_01_CLIENT_CONFIGURATION_HANDLE:
            size_used_or_needed = 2;
            memset(buffer, offset, buffer_size - offset);
            break;
        case ATT_CHARACTERISTIC_ed9da18c_a800_4f66_a670_aa7547e34453_01_VALUE_HANDLE:
            LOG_DEBUG("read fromnum - report most recently give number");
            size_used_or_needed = att_read_callback_handle_little_endian_32(from_num, offset, buffer, buffer_size);
            break;
        case ATT_CHARACTERISTIC_2c55e69e_4993_11ed_b878_0242ac120002_01_VALUE_HANDLE:
            memset(&buffer[offset], 0, buffer_size - offset);
            if (buffer_size == 0) {
                // LOG_DEBUG("read fromradio - pull data from radio");
                // A call with zero size buffer so read what we can send and find its size
                // Grab the next message in the queue or make empty if the queue is empty
                if (bluetoothPhoneAPI) {
                    size_used_or_needed = fromRadioNumBytes = bluetoothPhoneAPI->getFromRadio(fromRadioBytes);
                }
            } else if (fromRadioNumBytes > 0) {
                // LOG_DEBUG("read fromradio - pass along");
                // Second of a paired call so now we give the data we got last time
                memcpy(&buffer[offset], &fromRadioBytes, min(buffer_size - offset, fromRadioNumBytes));
                size_used_or_needed = fromRadioNumBytes;
            }
            break;
        case ATT_CHARACTERISTIC_f75c76d2_129e_4dad_a1dd_7866124401e7_01_VALUE_HANDLE:
            // LOG_DEBUG("read toradio - this should never happen");
            memset(buffer, offset, buffer_size - offset);
            break;
        case ATT_CHARACTERISTIC_5a3d6e49_06e6_4423_9944_e9de8cdf9547_01_VALUE_HANDLE:
            // LOG_DEBUG("read - A log message as LogRecord protobuf");
            memset(buffer, offset, buffer_size - offset);
            break;
        default:
            LOG_DEBUG("attempt to read undefined att_handle: %02x", att_handle);
            break;
    }
    // print_named_data("att read buffer out", buffer + offset, std::min(buffer_size, size_used_or_needed));
    return size_used_or_needed;
}

int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                       uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);
    // LOG_DEBUG("att_write_callback(0x%02x,0x%02x,%d,%d,buffer,%d)", connection_handle, att_handle, transaction_mode,  offset, buffer_size);
    // print_named_data("att write buffer in", buffer, buffer_size);

    if (transaction_mode != ATT_TRANSACTION_MODE_NONE) {
        switch (transaction_mode) {
            case ATT_TRANSACTION_MODE_NONE:
                break;
            case ATT_TRANSACTION_MODE_ACTIVE:
                LOG_DEBUG("att_write_callback - ATT_TRANSACTION_MODE_ACTIVE");
                break;
            case ATT_TRANSACTION_MODE_EXECUTE:
                LOG_DEBUG("att_write_callback - ATT_TRANSACTION_MODE_EXECUTE");
                break;
            case ATT_TRANSACTION_MODE_CANCEL:
                LOG_DEBUG("att_write_callback - ATT_TRANSACTION_MODE_CANCEL");
                break;
            case ATT_TRANSACTION_MODE_VALIDATE:
                LOG_DEBUG("att_write_callback - ATT_TRANSACTION_MODE_VALIDATE");
                break;
            default:
                break;
        }
        return 0;
    }

    switch (att_handle) {
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE:
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL_01_CLIENT_CONFIGURATION_HANDLE:
        case ATT_CHARACTERISTIC_ed9da18c_a800_4f66_a670_aa7547e34453_01_CLIENT_CONFIGURATION_HANDLE: {
            le_connection_t *context = connection_for_conn_handle(connection_handle, att_handle);
            context->notification_enabled = little_endian_read_16(buffer, 0) ==
                                           GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            context->connection_handle = connection_handle;
            if (context->notification_enabled) {
                att_server_request_can_send_now_event(connection_handle);
            }
            le_connection = context;
            break;
        }
        case ATT_CHARACTERISTIC_f75c76d2_129e_4dad_a1dd_7866124401e7_01_VALUE_HANDLE: {
            // LOG_DEBUG("write toradio");

            auto len = buffer_size - offset;
            // LOG_DEBUG("toRadioWriteCb data %p, len %u", &buffer[offset], len);
            if (memcmp(&lastToRadio, &buffer[offset], len) != 0) {
                // LOG_DEBUG("New ToRadio packet");
                memcpy(&lastToRadio, &buffer[offset], len);
                if (bluetoothPhoneAPI) {
                    bluetoothPhoneAPI->handleToRadio(&buffer[offset], len);
                }
            } else {
                // LOG_DEBUG("Drop dup ToRadio packet we just saw");
            }
            break;
        }
        default:
            LOG_DEBUG("BLE Service: Unsupported write to 0x%02x, len %u", att_handle, buffer_size);
            break;
    }
    return 0;
}

void setup_advertisements() {
    //ADV_IND - It allows for up to 31 bytes of advertising data in the main packet payload.
    static uint8_t adv_data[BLE_ADVERTISING_MAX_LENGTH];
    uint8_t adv_data_len = 0;

    //0xXX (the length of the subsequent data, {d0(type), d01-dXX}
    adv_data[adv_data_len++] = 0x2;
    adv_data[adv_data_len++] = BLUETOOTH_DATA_TYPE_FLAGS;
    adv_data[adv_data_len++] = BLE_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    auto name = getDeviceName();
    uint8_t name_len = strlen(name);

    auto fitable_name_len = min(BLE_ADVERTISING_MAX_LENGTH - adv_data_len - 2, name_len);
    adv_data[adv_data_len++] = fitable_name_len + 1;
    if (fitable_name_len == name_len) {
        adv_data[adv_data_len++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    } else {
        adv_data[adv_data_len++] = BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME;
    }
    memcpy(&adv_data[adv_data_len], name, fitable_name_len);
    adv_data_len += fitable_name_len;

    // Uncertain whether we have to supply the Meshtastic 128bit UUID for things to work,
    // it's not needed for Android as a samsung bug in a specific phone has the android code
    // just useing the name matching the tail of the nodeid

    // For Bluetooth Low Energy, the radio is periodically used to broadcast advertisements that are used for
    // both discovery and connection establishment.

    // setup advertisements - units each of 625us
    uint16_t adv_int_min = 1000000 / 625; // 1 second
    uint16_t adv_int_max = 2000000 / 625; // 2 seconds
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, adv_data);
    gap_scan_response_set_data(scan_response_meshtastic_data_length, scan_response_meshtastic_data);
    gap_advertisements_enable(1);
}

void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    // LOG_DEBUG("hci_packet_handler(0x%02x,%d,data,%d)", packet_type, channel, size);
    // print_named_data("hci_packet", packet, size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    // LOG_DEBUG("event_type: 0x%02x", event_type);
    switch (event_type) {
        case BTSTACK_EVENT_STATE: //0x60
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            LOG_DEBUG("BTstack up and running on: %s", bd_addr_to_str(local_addr));
            setup_advertisements();
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE: //0x05
            // LOG_DEBUG("tidying connection: 0x%02x, 0x%02x, enabled: %d", le_connection->connection_handle, le_connection->value_handle, le_connection->notification_enabled);
            //le_connection.notification_enabled = 0;
            LOG_DEBUG("LE Connection 0x%04x : disconnect, reason 0x%02x", le_connection->connection_handle, hci_event_disconnection_complete_get_reason(packet));
            break;

        case SM_EVENT_PAIRING_STARTED: //0xd4
            LOG_DEBUG("Pairing Started");
            break;
        case SM_EVENT_PAIRING_COMPLETE: //0xd5
            LOG_DEBUG("Pairing Complete");
            break;
        case BTSTACK_EVENT_NR_CONNECTIONS_CHANGED: {
            //0x61
            const uint8_t connections = btstack_event_nr_connections_changed_get_number_connections(packet);
            LOG_DEBUG("Number of connections changed: %d", connections);
            break;
        }

        default:
            break;
    }
}

void att_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    // LOG_DEBUG("att_packet_handler(0x%02x,%d,data,%d)", packet_type, channel, size);
    if (packet_type != HCI_EVENT_PACKET) return;

    // print_named_data("att_packet", packet, size);

    uint8_t event_type = hci_event_packet_get_type(packet);
    // LOG_DEBUG("event_type: 0x%02x", event_type);
    switch (event_type) {
        case ATT_EVENT_CAN_SEND_NOW: //0xb7
            if (le_connection->notification_enabled) {
                // LOG_DEBUG("att_server_notify(0x%02x,0x%02x,%d,%d)", le_connection->connection_handle, le_connection->value_handle, le_connection->data, le_connection->data_size);
                att_server_notify(le_connection->connection_handle, le_connection->value_handle,
                                  (uint8_t *) &le_connection->data, le_connection->data_size);
            }

            break;
        case ATT_EVENT_CONNECTED: //0xb3
            LOG_DEBUG("Connected");
            le_connection->connected = true;
            break;
        case ATT_EVENT_DISCONNECTED: //0xb4
            LOG_DEBUG("Disconnected");
            le_connection->connected = false;
            break;

        default:
            break;
    }
}

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    // LOG_DEBUG("sm_packet_handler(0x%02x,%d,data,%d)", packet_type, channel, size);
    if (packet_type != HCI_EVENT_PACKET) return;

    // print_named_data("sm_packet", packet, size);

    uint8_t event_packet_type = hci_event_packet_get_type(packet);
    // LOG_DEBUG("event_packet_type: 0x%02x", event_packet_type);
    switch (event_packet_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            LOG_DEBUG("Just works requested");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            LOG_DEBUG("Confirming numeric comparison: %d", sm_event_numeric_comparison_request_get_passkey(packet));
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            LOG_INFO("Display Passkey: %d", sm_event_passkey_display_number_get_passkey(packet));
            break;
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            LOG_DEBUG("Identity Resolving Started");
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            LOG_DEBUG("Identity Resolving Failed");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            switch (sm_event_pairing_complete_get_status(packet)) {
                case ERROR_CODE_SUCCESS:
                    LOG_DEBUG("Pairing complete, success");
                    connection_secured = true;
                    break;
                case ERROR_CODE_CONNECTION_TIMEOUT:
                    LOG_DEBUG("Pairing failed, timeout");
                    break;
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    LOG_DEBUG("Pairing failed, disconnected");
                    break;
                case ERROR_CODE_AUTHENTICATION_FAILURE:
                    LOG_DEBUG("Pairing failed, reason = %u", sm_event_pairing_complete_get_reason(packet));
                    break;
                default:
                    break;
            }
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            LOG_DEBUG("Re-encryption complete, success");
            connection_secured = true;
            break;
        default:
            break;
    }

    if (connection_secured) {
        // continue - query primary services
        LOG_INFO("BLE pair success");
        bluetoothStatus->updateStatus(
            new meshtastic::BluetoothStatus(meshtastic::BluetoothStatus::ConnectionState::CONNECTED));
    } else if (event_packet_type == SM_EVENT_PAIRING_COMPLETE) {
        LOG_INFO("BLE pair failed");
        // Notify UI (or any other interested firmware components)
        bluetoothStatus->updateStatus(
            new meshtastic::BluetoothStatus(meshtastic::BluetoothStatus::ConnectionState::DISCONNECTED));
    }
}




PicoWBluetooth::~PicoWBluetooth() {
    delete bluetoothPhoneAPI;
    bluetoothPhoneAPI = nullptr;
}

void printAvailableLogging();

void PicoWBluetooth::setup() {
    __lockBluetooth();

    if (!bluetoothPhoneAPI) {
        bluetoothPhoneAPI = new BluetoothPhoneAPI();
    }

    memset(lastToRadio, 0, MAX_TO_FROM_RADIO_SIZE);

    l2cap_init();
    sm_init();
    if (config.bluetooth.mode == meshtastic_Config_BluetoothConfig_PairingMode_FIXED_PIN) {
        sm_use_fixed_passkey_in_display_role(config.bluetooth.fixed_pin);
        LOG_DEBUG("Bluetooth pin set to '%d'", config.bluetooth.fixed_pin);
    } else {
        LOG_DEBUG("Bluetooth pin will be generated on the fly as needed");
    }
#ifdef ARDUINO_ARCH_RP2040
    printAvailableLogging();
#endif
    if (config.has_display) {
        sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_ONLY);
    }

    // LOG_DEBUG("att_server_init()");
    att_server_init(profile_data, att_read_callback, att_write_callback);

    // LOG_DEBUG("inform about BTstack state");
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // LOG_DEBUG("inform about security manager state");
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    // LOG_DEBUG("register for ATT event");
    att_server_register_packet_handler(att_packet_handler);
    hci_power_control(HCI_POWER_ON);

#ifdef ARDUINO_ARCH_RP2040
    printAvailableLogging();
#endif

    __unlockBluetooth();
}

void PicoWBluetooth::shutdown() {
    LOG_INFO("Bluetooth shutdown");
    __lockBluetooth();
    if (bluetoothPhoneAPI) {
        bluetoothPhoneAPI->close();
        delete bluetoothPhoneAPI;
        bluetoothPhoneAPI = nullptr;
    }
    connection_secured = false;
    hci_power_control(HCI_POWER_OFF);
    __unlockBluetooth();
}

void PicoWBluetooth::restart() {
    __lockBluetooth();
    memset(lastToRadio, 0, MAX_TO_FROM_RADIO_SIZE);

    hci_power_control(HCI_POWER_ON);
    __unlockBluetooth();
}

void PicoWBluetooth::clearBonds() {
    LOG_ERROR("PicoWBluetooth::clearBonds - not implemented");
}

bool PicoWBluetooth::isConnected() {
    //see also BluetoothPhoneAPI::checkIsConnected()
    return le_connection->connected || le_connection->notification_enabled || connection_secured;
}

int PicoWBluetooth::getRssi() {
    return 0;
}

void PicoWBluetooth::updateBatteryLevel(uint8_t level) {
    battery_level = level;
}
