#include "bluetooth.h"
#include "linear.h"

bool   deviceConnected = false;
String receivedData    = "";

// ── Callbacks ────────────────────────────────────────────────────────────────

void MyServerCallbacks::onConnect(BLEServer*) {
    deviceConnected = true;
    if (DEBUG_BLE) Serial.println(">> DISPOSITIVO CONECTADO <<");
}

void MyServerCallbacks::onDisconnect(BLEServer*) {
    deviceConnected = false;
    if (DEBUG_BLE) Serial.println(">> DISPOSITIVO DESCONECTADO <<");
    BLEDevice::startAdvertising();
}

void MyCallbacks::onWrite(BLECharacteristic* pCharacteristic) {
    auto   tmp     = pCharacteristic->getValue();
    String rxValue = String(tmp.c_str());
    if (rxValue.length() > 0)
        receivedData = rxValue;
}

// ── BluetoothCommunication ───────────────────────────────────────────────────

void BluetoothCommunication::begin() {
    BLEDevice::init(device_name.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pService = pServer->createService(SERVICE_UUID);

    pCharTX = pService->createCharacteristic(
        CHAR_TX_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pCharTX->addDescriptor(new BLE2902());

    pCharRX = pService->createCharacteristic(
        CHAR_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharRX->setCallbacks(new MyCallbacks());

    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();

    if (DEBUG_BLE)
        Serial.println("BLE pronto: TX e RX configurados");
}

void BluetoothCommunication::sendData(const char* data) {
    if (!deviceConnected || !pCharTX)
        return;
    size_t len = strlen(data);
    if (len > TELEMETRY_PART_LEN)
        len = TELEMETRY_PART_LEN;
    pCharTX->setValue((uint8_t*)data, len);
    pCharTX->indicate();
}

void BluetoothCommunication::updateTelemetryCache() {
    telemetry_data.header              = 0xEFBE;
    telemetry_data.encoder_left_count  = linearCar.getLimitePassos(); // limite de passos 
    telemetry_data.encoder_right_count = 0;
    telemetry_data.left_velocity_ms    = 0;
    telemetry_data.right_velocity_ms   = 0;
    telemetry_data.left_velocity_target  = 0;
    telemetry_data.right_velocity_target = 0;
    telemetry_data.left_motor_pwm  = 0;
    telemetry_data.right_motor_pwm = 0;
    telemetry_data.left_gain  = 0;
    telemetry_data.right_gain = 0;

    telemetry_data.odom_x = linearCar.getSteps(); // passo real
    telemetry_data.odom_y = linearCar.getAbsolutePosition(); // posiçao absoluta em mm,
    telemetry_data.odom_theta = linearCar.getRelativePosition(); // posiçao relativa em mm, considerando o ponto de homing como zero

    telemetry_data.odom_vx    = 0;
    telemetry_data.odom_vy    = 0;
    telemetry_data.odom_omega = 0;
    telemetry_data.accel_x    = 0;
    telemetry_data.accel_y    = 0;
    telemetry_data.accel_z    = 0;

    telemetry_data.line_distance_mm       = 0;
    telemetry_data.line_marker_count      = 0;
    telemetry_data.line_marker_distance_m = 0;

    telemetry_data.gps_latitude  = 0;
    telemetry_data.gps_longitude = 0;
    telemetry_data.gps_altitude  = 0;
    telemetry_data.gps_speed     = 0;
    telemetry_data.gps_valid     = 0;

    memset(telemetry_data.rfid_uid, 0, sizeof(telemetry_data.rfid_uid));

    telemetry_cache_binary_len = sizeof(TelemetryData);
}

String BluetoothCommunication::receiveData() {
    String temp = receivedData;
    receivedData = "";
    return temp;
}

void BluetoothCommunication::processCommand(String command) {
    command.trim();
    command.toUpperCase();

    if (command == "START") {
        if (DEBUG_BLE) Serial.println("Comando: Iniciar movimento");
    } else if (command.startsWith("VEL:")) {
        String valueStr = command.substring(4);
        valueStr.trim();
        int splitPos = valueStr.indexOf(' ');
        if (splitPos >= 0)
            valueStr = valueStr.substring(0, splitPos);
        float velocity = valueStr.toFloat();
        if (velocity > 0)
            linearCar.setBypassControl(INDO);
        else if (velocity < 0)
            linearCar.setBypassControl(VOLTANDO);
        else
            linearCar.setBypassControl(PARADO);
    } else if (command == "FOLLOW_LINE_START") {
        linearCar.setCommand(PLAY);
    } else if (command == "FOLLOW_LINE_STOP") {
        linearCar.setCommand(STOP);
    } else if (command == "VOLTA_ZERO") {
        if (DEBUG_BLE) Serial.println("Comando: Homing");
        linearCar.requestHoming();
    }
}

void BluetoothCommunication::handler() {
    String bleData = receiveData();

    if (bleData.length() > 0) {
        if (bleData.startsWith("CMD:")) {
            String command = bleData.substring(4);
            if (DEBUG_BLE) Serial.println("Comando recebido: " + command);
            processCommand(command);
        } else if (bleData.startsWith("VEL:")) {
            processCommand(bleData);
        } else if (bleData.startsWith("RQS")) {
            updateTelemetryCache();
            telemetry_data.esp_timestamp_ms = millis();
            pCharTX->setValue((uint8_t*)&telemetry_data, sizeof(TelemetryData));
            pCharTX->indicate();
        } else {
            if (DEBUG_BLE) Serial.println("Comando direto: " + bleData);
            processCommand(bleData);
        }
    }

    static unsigned long lastTelemetryTime = 0;
    unsigned long currentMillis = millis();
    if (deviceConnected && (currentMillis - lastTelemetryTime >= 300)) {
        lastTelemetryTime = currentMillis;
        updateTelemetryCache();
        if (telemetry_cache_binary_len > 0) {
            telemetry_data.esp_timestamp_ms = millis();
            pCharTX->setValue((uint8_t*)&telemetry_data, sizeof(TelemetryData));
            pCharTX->indicate();
        }
    }
}
