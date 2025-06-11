#include <ArduinoBLE.h>
#include <EEPROM.h>
#include <LSM6DS3.h>
#include <Wire.h>

// IMU initialization
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Threshold and sample config
const float accThresh = 3.5;
const int numSamples = 119;
int samplesRead = numSamples;

// EEPROM addresses
const int stat_addr = 0;
const int step_addr = 4;

// Device status and step counter
int dev_stat;
int steps = 0;

// BLE Services and Characteristics
BLEService deviceInfoService("180A");    // Device Info
BLEService statusService("1234");        // Device status
BLEService commandService("1818");       // Command
BLEService stepService("1826");          // Step tracking

BLECharacteristic devID("0x2A00", BLERead | BLENotify, 512);       // Device ID
BLECharacteristic devStatus("0x2ADA", BLERead | BLENotify, 512);   // Device status
BLECharacteristic commandBLE("0x2A9F", BLERead | BLEWrite, 512);   // Control command
BLECharacteristic stepsCount("0x2ACF", BLERead | BLEWrite | BLENotify, 512);  // Step count

// Timing
unsigned long previousMillis = 0;
const long interval = 5000;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("BLE start failed!");
    while (1);
  }

  // Device name
  BLE.setLocalName("InsoleTest_pair2");
  BLE.setDeviceName("InsoleTest_pair2");

  // Add characteristics to services
  deviceInfoService.addCharacteristic(devID);
  statusService.addCharacteristic(devStatus);
  commandService.addCharacteristic(commandBLE);
  stepService.addCharacteristic(stepsCount);

  // Add services
  BLE.addService(deviceInfoService);
  BLE.addService(statusService);
  BLE.addService(commandService);
  BLE.addService(stepService);

  // Event handler for command
  commandBLE.setEventHandler(BLEWritten, wait_command);

  // Start advertising
  BLE.advertise();
  Serial.println("Waiting for connections...");

  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("IMU ERROR");
  } else {
    Serial.println("IMU OK!");
  }

  // Set device ID
  String dev_ID = BLE.address();
  byte dev_id_b[dev_ID.length() + 1];
  dev_ID.getBytes(dev_id_b, dev_ID.length() + 1);
  devID.writeValue(dev_id_b, sizeof(dev_id_b), false);

  // Read and check EEPROM
  checkEEPROM();
  update_status();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("CONNECTED TO MAIN DEVICE");
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      if (dev_stat == 1) {
        do_activity();
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
    systemReset();
  }

  if (dev_stat == 1) {
    do_activity();
  }
}

void do_activity() {
  unsigned long currentMillis = millis();

  float aX = myIMU.readFloatAccelX();
  float aY = myIMU.readFloatAccelY();
  float aZ = myIMU.readFloatAccelZ();

  float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

  if (aSum >= accThresh) {
    samplesRead = 0;
    steps++;
    Serial.println("MOVEMENT");

    while (samplesRead < numSamples) {
      myIMU.readFloatAccelX();
      myIMU.readFloatAccelY();
      myIMU.readFloatAccelZ();
      samplesRead++;
    }
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    update_act();
  }
}

void wait_command(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t receivedData[1];
  uint8_t stat = 9;
  int bytesRead = commandBLE.readValue(receivedData, sizeof(receivedData));

  if (bytesRead > 0) {
    stat = receivedData[0];
  }

  Serial.print("Command: ");
  Serial.println(stat);

  if (stat != dev_stat) {
    switch (stat) {
      case 1: // START
        Serial.println("START ACTIVITY");
        EEPROM.put(stat_addr, 1);
        update_status();
        break;

      case 2: // SYNC
        Serial.println("SYNC DATA");
        for (int i = 0; i < 350; i++) {
          EEPROM.write(i, 0);
        }
        steps = 0;
        update_status();
        break;

      default:
        Serial.println("UNKNOWN COMMAND");
        break;
    }
  }
}

void update_status() {
  EEPROM.get(stat_addr, dev_stat);
  Serial.print("CURRENT STATUS: ");
  Serial.println(dev_stat);

  byte data_to_send[1];
  data_to_send[0] = dev_stat;
  devStatus.writeValue(data_to_send, sizeof(data_to_send), false);
}

void update_act() {
  EEPROM.put(step_addr, steps);

  char step_c[10];
  itoa(steps, step_c, 10);
  stepsCount.writeValue(step_c, strlen(step_c), false);

  Serial.print("STEPS: ");
  Serial.println(steps);
}

void checkEEPROM() {
  EEPROM.get(stat_addr, dev_stat);
  if (dev_stat != 0 && dev_stat != 1) {
    dev_stat = 0;
    EEPROM.put(stat_addr, dev_stat);
  }

  EEPROM.get(step_addr, steps);
  if (steps < 0 || steps > 100000) {
    steps = 0;
    EEPROM.put(step_addr, steps);
  }
}


