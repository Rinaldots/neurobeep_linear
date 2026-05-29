#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID  "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
#define CHAR_RX_UUID  "d5593e6b-3328-493a-b3c9-9814683d8e40"
#define CHAR_TX_UUID  "12345678-1234-5678-1234-56789abcdef0"

#ifndef DEBUG_BLE
#define DEBUG_BLE 1
#endif

#ifndef TELEMETRY_PART_LEN
#define TELEMETRY_PART_LEN 130
#endif

struct __attribute__((packed)) TelemetryData {
    uint16_t header;
    int32_t  encoder_left_count;
    int32_t  encoder_right_count;
    float    left_velocity_ms;
    float    right_velocity_ms;
    float    left_velocity_target;
    float    right_velocity_target;
    float    left_motor_pwm;
    float    right_motor_pwm;
    float    left_gain;
    float    right_gain;
    float    odom_x;
    float    odom_y;
    float    odom_theta;
    float    odom_vx;
    float    odom_vy;
    float    odom_omega;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    int16_t  line_distance_mm;
    uint16_t line_marker_count;
    float    line_marker_distance_m;
    float    gps_latitude;
    float    gps_longitude;
    float    gps_altitude;
    float    gps_speed;
    uint8_t  gps_valid;
    char     rfid_uid[12];
    uint32_t esp_timestamp_ms;
};

extern bool   deviceConnected;
extern String receivedData;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override;
};

class BluetoothCommunication {
public:
    void   begin();
    void   sendData(const char* data);
    void   updateTelemetryCache();
    String receiveData();
    void   processCommand(String command);
    void   handler();
    bool   isConnected() const { return deviceConnected; }

    TelemetryData telemetry_data{};
    String        device_name = "Neuro_Robot";

private:
    BLEServer*         pServer  = nullptr;
    BLECharacteristic* pCharTX  = nullptr;
    BLECharacteristic* pCharRX  = nullptr;
    BLEService*        pService = nullptr;

    size_t telemetry_cache_binary_len = 0;
};

#endif
