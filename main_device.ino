#include <ArduinoBLE.h>
#include <EEPROM.h>
#include <LSM6DS3.h>
#include <Wire.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Thresholds and sampling
const float accThresh = 3.5;
const int numSamples = 119;
int samplesRead = numSamples;
const float impactThreshold = 8.0;

// EEPROM addresses
const int usr_id = 0;
const int stat_addr = 4;
const int p_stat_addr = 8;
const int p_devID = 12;
const int date_addr = 16;
const int time_addr = 20;
const int step_addr = 24;
const int impact_addr = 28;  // ➕ Added for impact count

// BLE pairing
bool paired = false;
String pair_addr = "58:3b:c2:55:bc:21";

// Device state and steps
int dev_stat;
int steps = 0;
int impactCount = 0;  // ➕ Track impacts

// BLE Service and Characteristics
BLEService deviceService("180A");

BLECharacteristic devID("0x2A00", BLERead | BLENotify, 512);
BLECharacteristic pdevID("0x2A01", BLERead | BLENotify, 512);
BLECharacteristic devStatus("0x2ADA", BLERead | BLENotify, 512);
BLECharacteristic pdevStatus("0x2ADB", BLERead | BLENotify, 512);
BLECharacteristic impactEvent("0x2B3F", BLERead | BLEWrite | BLENotify, 512);
BLECharacteristic commandBLE("0x2A9F", BLERead | BLEWrite, 512);
BLECharacteristic stepsCount("0x2ACF", BLERead | BLEWrite | BLENotify, 512);
BLECharacteristic rawAccelData("0x2B40", BLERead | BLENotify, 512); // New UUID and size
BLECharacteristic rawGyroChar("0x2B41", BLERead | BLENotify, 512); // Custom UUID for gyro
BLECharacteristic ImpactStrengthChar("0x2B43", BLERead | BLENotify,512);





BLEDevice secondDevice;
BLECharacteristic secondChar;
BLECharacteristic secondStepsChar;
BLECharacteristic secondStatusChar;

unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;
unsigned long previousMillis = 0;
const long interval = 5000;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("InsoleTest2");
  BLE.setDeviceName("InsoleTest2");
  BLE.setAdvertisedService(deviceService);

  deviceService.addCharacteristic(devID);
  deviceService.addCharacteristic(pdevID);
  deviceService.addCharacteristic(devStatus);
  deviceService.addCharacteristic(pdevStatus);
  deviceService.addCharacteristic(commandBLE);
  deviceService.addCharacteristic(stepsCount);
  deviceService.addCharacteristic(impactEvent);
  deviceService.addCharacteristic(rawAccelData);
  deviceService.addCharacteristic(rawGyroChar);
  deviceService.addCharacteristic(ImpactStrengthChar);


  commandBLE.setEventHandler(BLEWritten, wait_command);

  BLE.addService(deviceService);
  BLE.advertise();
  Serial.println("Waiting for connections...");

  if (myIMU.begin() != 0) {
    Serial.println("IMU ERROR");
  } else {
    Serial.println("IMU OK!");
  }

  // Write device ID
  String dev_ID = BLE.address();
  byte dev_id_b[dev_ID.length() + 1];
  dev_ID.getBytes(dev_id_b, dev_ID.length() + 1);
  devID.writeValue(dev_id_b, sizeof(dev_id_b), false);

  // Load status and data from EEPROM
  update_status();
  EEPROM.get(step_addr, steps);
  EEPROM.get(impact_addr, impactCount);  // ➕ Load impact count
  Serial.print("Loaded Steps: ");
  Serial.println(steps);
  Serial.print("Loaded Impact Count: ");
  Serial.println(impactCount);

  connectToSecondDevice();
}

void loop() {
  BLE.poll();

  if (commandBLE.written()) {
    forwardCommandToSecondDevice();
  }

  if (!secondDevice || !secondDevice.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > reconnectInterval) {
      Serial.println("Attempting to reconnect to SecondDevice...");
      connectToSecondDevice();
      lastReconnectAttempt = now;
    }
  }

  BLEDevice central = BLE.central();

  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Connected to app");

    while (central.connected()) {
      if (dev_stat == 1) {
        do_activity();
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
  }

  if (dev_stat == 1) {
    do_activity();
  }
}
void sendRawAccelData() {
  float x = myIMU.readFloatAccelX();
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();

  // Format: X:1.23,Y:4.56,Z:7.89
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "X:%.2f,Y:%.2f,Z:%.2f", x, y, z);
  
  rawAccelData.writeValue((uint8_t*)buffer, strlen(buffer), false);

  Serial.print("Raw Accel: ");
  Serial.println(buffer);
}
void sendRawGyroData() {
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  // Format: GX:1.23,GY:4.56,GZ:7.89
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "GX:%.2f,GY:%.2f,GZ:%.2f", gx, gy, gz);

  rawGyroChar.writeValue((uint8_t*)buffer, strlen(buffer), false);

  Serial.print("Raw Gyro: ");
  Serial.println(buffer);
}

void sendImpactStrength(float impactStrength) {
  // Format: Impact:8.75
  char buffer[30];
  snprintf(buffer, sizeof(buffer), "%.2f", impactStrength);

  ImpactStrengthChar.writeValue((uint8_t*)buffer, strlen(buffer), false);

  Serial.print("Impact Strength: ");
  Serial.println(buffer);
}


void do_activity() {
  unsigned long currentMillis = millis();
  float aX = myIMU.readFloatAccelX();
  float aY = myIMU.readFloatAccelY();
  float aZ = myIMU.readFloatAccelZ();
  float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

  if (aSum >= impactThreshold) {
    impactCount++;
    Serial.print("IMPACT DETECTED - Count: ");
    Serial.println(impactCount);
    sendImpactStrength(aSum);
  }

  if (aSum >= accThresh) {
    samplesRead = 0;
    Serial.println("MOVEMENT");
    steps++;
  }

  while (samplesRead < numSamples) {
    myIMU.readFloatAccelX();
    myIMU.readFloatAccelY();
    myIMU.readFloatAccelZ();
    samplesRead++;
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendRawAccelData(); 
    sendRawGyroData();
    update_act();
  }
}

void wait_command(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t receivedData[14];
  uint8_t stat = 9;
  uint32_t dt = 0;
  uint32_t ts = 0;

  int bytesRead = commandBLE.readValue(receivedData, sizeof(receivedData));
  if (bytesRead > 0) {
    stat = receivedData[0];
    if (stat == 1) {
      dt = (receivedData[1] * 10000) + (receivedData[2] * 100) + receivedData[3];
      ts = (receivedData[4] * 10000) + (receivedData[5] * 100) + receivedData[6];
    }

    Serial.print("Command: ");
    Serial.println(stat);
    Serial.print("Date: ");
    Serial.println(dt);
    Serial.print("Time: ");
    Serial.println(ts);

    if (stat != dev_stat) {
      switch (stat) {
        case 1:
          Serial.println("START ACTIVITY");
          EEPROM.put(stat_addr, 1);
          EEPROM.put(date_addr, dt);
          EEPROM.put(time_addr, ts);
          update_status();
          
          break;

        case 2:
          Serial.println("SYNC DATA");
          for (int i = 0; i < 350; i++) EEPROM.write(i, 0);
          steps = 0;
          impactCount = 0;  // ➕ Reset impact count
          update_status();
          break;
      }
    }

    forwardCommandToSecondDevice();
  }
}

void update_status() {
  EEPROM.get(stat_addr, dev_stat);
  uint32_t dtt = 0;
  uint32_t tss = 0;
  EEPROM.get(date_addr, dtt);
  EEPROM.get(time_addr, tss);

  byte data_to_send[7];
  data_to_send[0] = dev_stat;
  data_to_send[1] = (dtt / 10000) % 100;
  data_to_send[2] = (dtt / 100) % 100;
  data_to_send[3] = dtt % 100;
  data_to_send[4] = (tss / 10000) % 100;
  data_to_send[5] = (tss / 100) % 100;
  data_to_send[6] = tss % 100;

  devStatus.writeValue(data_to_send, sizeof(data_to_send), false);

  Serial.print("STATUS: ");
  Serial.println(dev_stat);
  Serial.print("DATE: ");
  Serial.println(dtt);
  Serial.print("TIME: ");
  Serial.println(tss);
}

void update_act() {
  int secondSteps = 0;

  if (secondDevice && secondDevice.connected() && secondStepsChar) {
    if (secondStepsChar.read()) {
      const uint8_t* value = secondStepsChar.value();
      int len = secondStepsChar.valueLength();

      char buffer[20] = { 0 };
      memcpy(buffer, value, min(len, sizeof(buffer) - 1));
      secondSteps = String(buffer).toInt();

      Serial.print("Second Device Steps: ");
      Serial.println(secondSteps);
    } else {
      Serial.println("Failed to read steps from second device.");
    }
  }

  int totalSteps = steps + secondSteps;

  EEPROM.put(step_addr, totalSteps);
  EEPROM.put(impact_addr, impactCount);  // ➕ Save impact count to EEPROM

  char step_c[10];
  itoa(totalSteps, step_c, 10);
  stepsCount.writeValue(step_c, strlen(step_c), false);

  char impact_c[10];
  itoa(impactCount, impact_c, 10);
  impactEvent.writeValue(impact_c, strlen(impact_c), false);

  Serial.print("Total Steps (Main + Second): ");
  Serial.println(totalSteps);
  Serial.print("Impact Count: ");
  Serial.println(impactCount);
}

void forwardCommandToSecondDevice() {
  uint8_t data[7] = { 0 };
  int len = commandBLE.valueLength();
  const uint8_t* raw = commandBLE.value();

  for (int i = 0; i < len && i < 7; i++) {
    data[i] = raw[i];
  }

  Serial.print("App sent (with default fallback): ");
  for (int i = 0; i < 7; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (secondDevice && secondDevice.connected() && secondChar) {
    secondChar.writeValue(data, 7);
    Serial.println("Forwarded to second device.");
  } else {
    Serial.println("Second device not connected.");
  }
}

void connectToSecondDevice() {
  const char* targetAddress = "58:3b:c2:55:bc:21";
  Serial.print("Scanning for device with MAC: ");
  Serial.println(targetAddress);

  BLE.scanForAddress(targetAddress);
  unsigned long scanStart = millis();
  bool found = false;

  while (millis() - scanStart < 30000) {
    BLEDevice peripheral = BLE.available();

    if (peripheral) {
      Serial.print("Found device: ");
      Serial.println(peripheral.address());

      if (strcmp(peripheral.address().c_str(), targetAddress) == 0) {
        Serial.println("MAC address match! Connecting...");
        int retries = 0;
        bool connected = false;
        while (retries < 3 && !connected) {
          if (peripheral.connect()) {
            Serial.println("Connected to SecondDevice");
            secondDevice = peripheral;
            found = true;
            connected = true;
            break;
          } else {
            retries++;
            Serial.print("Retrying connection... (");
            Serial.print(retries);
            Serial.println("/3)");
          }
        }

        if (connected) {
          if (secondDevice.discoverAttributes()) {
            secondChar = peripheral.characteristic("00000000-0000-0000-0000-000000002a9f");
            secondStepsChar = peripheral.characteristic("00000000-0000-0000-0000-000000002acf");   // stepsCount
            secondStatusChar = peripheral.characteristic("00000000-0000-0000-0000-000000002ada");  // devStatus

            if (!secondChar || !secondStepsChar || !secondStatusChar) {
              Serial.println("Required characteristic not found.");
            } else {
              Serial.println("All characteristics found.");
            }
          } else {
            Serial.println("Failed to discover services/characteristics.");
          }
        }
        break;
      }
    }

    delay(100);
  }

  BLE.stopScan();

  if (!found) {
    Serial.println("SecondDevice not found during scan.");
  }
}