/**
 * @file sicura.ino
 * @brief source file for sicura system firmware
 * @version 0.1
 * @date 2023-03-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/**
  libraries
**/
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>

#include <LSM6DS3.h>
#include <Wire.h>

#include <bluefruit.h>

// instance of class MAX3010X
MAX30105 PPG;

byte ledBrightness = 0x2A; //Options: 0=Off to 255=50mA
byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

// instance of class LSM6DS3
LSM6DS3 IMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
float aX, aY, aZ;
const float crash_threshold = 32; // threshold of significant in G's
uint8_t crash_detected = 0; // flag to hold crash detection status

// bluetooth setup
BLECharacteristic hrchr = BLECharacteristic(0xABC0); // heartrate characteristic
BLECharacteristic stchr = BLECharacteristic(0xABC1); // step count characteristic
BLECharacteristic btchr = BLECharacteristic(0xABC2); // heartrate characteristic
BLECharacteristic crchr = BLECharacteristic(0xABC3); // fall detection characteristic

BLEDis bledis; // device descriptor 

// battery percent value
uint8_t battery_value = 0;

// misc 
long last_update;

void setup() {
  Serial.begin(115200);

  // Configure the IMU
  if (IMU.begin() != 0){
    Serial.println("IMU failed to be configured");
    while(1);
  }

  // Init the PPG sensor
  if (!PPG.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("PPG failed to be Initialized");
    while (1);
  }  
  else{
    PPG.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  }

  // enable step counter:
  if(0 != config_pedometer(true)){
    Serial.println("Error Setting up step counter");
  }

  // init the BLE subsystem
  Bluefruit.begin();
  Bluefruit.setName("sicura");

  // peripheral callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // ble descriptor setup
  bledis.setManufacturer("sicura");
  bledis.setModel("band");
  bledis.begin();

  // hr characteristic setup
  hrchr.setProperties(CHR_PROPS_READ);
  hrchr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrchr.begin();

  // step count setup
  stchr.setProperties(CHR_PROPS_READ);
  stchr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  stchr.begin();

  // battery characteristic setup
  btchr.setProperties(CHR_PROPS_READ);
  btchr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  btchr.begin();

  // fall detection characteristic setup
  crchr.setProperties(CHR_PROPS_READ);
  crchr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  crchr.begin();

  // begin advertising
  bleAdvertise();

  // setup battery level detection:
  pinMode(PIN_VBAT, INPUT);

  last_update = millis();
}

void loop(){
  digitalToggle(LED_RED);

  uint8_t dataByte = 0;
  uint16_t stepCount = 0;  

  // handle data from the heart rate sensor
  long irValue = PPG.getIR();
  uint8_t hr = irValue/1000;

  // handle data from the accelerometer and compute the total G force exerted
  aX = IMU.readFloatAccelX();
  aY = IMU.readFloatAccelY();
  aZ = IMU.readFloatAccelZ();

  float acc_sum = fabs(aX) + fabs(aY) + fabs(aZ);

  // parse analog read of the battery ADC
  float bat_pr = analogRead(PIN_VBAT)/1.65;
  battery_value = (uint8_t)bat_pr;

  // handle crash condition from accelerometer shock values
  if(acc_sum >= crash_threshold) crash_detected = 1;

  // handle data from step counter registers
  IMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepCount = (dataByte << 8) & 0xFFFF;

  IMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepCount |=  dataByte;

  if (millis() - last_update > 1000) {
    last_update = millis();
    if (irValue < 45000) Serial.println("sensor not placed on wrist/hand");
    if(Bluefruit.connected()){
      uint8_t hrdata[1] = {hr};
      uint16_t stdata[1] = {stepCount};
      uint8_t btdata[1] = {battery_value};
      uint8_t crdata[1] = {crash_detected};

      // write the heart rate data to the characteristic
      if(hrchr.write(hrdata, sizeof(hrdata))){
        Serial.print("Heart Rate Measurement updated to: "); 
        Serial.println(hr); 
      }
      else{
        Serial.println("HR ERROR!");
      }

      // write the step count data to the characteristic
      if(stchr.write(stdata, sizeof(stdata))){
        Serial.print("Step Count Measurement updated to: "); 
        Serial.println(stepCount); 
      }
      else{
        Serial.println("Step Count ERROR!");
      }

      // write the battery data to the characteristic
      if(btchr.write(btdata, sizeof(hrdata))){
        Serial.print("Battery Measurement updated to: "); 
        Serial.println(battery_value); 
      }
      else{
        Serial.println("Bat ERROR!");
      }

      // write the crash data to the characteristic
      if(crchr.write(crdata, sizeof(crdata))){
          Serial.print("Crash Measurement updated to: "); 
          Serial.println(crash_detected);
      }
      else{
        Serial.println("Crash ERROR!");
      }
    }
  }    
}

void connect_callback(uint16_t conn_handle){
  Serial.println("Connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason){
  (void)conn_handle;
  (void)reason;

  Serial.println("Disconnected");

}

void bleAdvertise(void){  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// Setup pedometer mode for the IMU
int config_pedometer(bool clearStep) {
    uint8_t errorAccumulator = 0;
    uint8_t dataToWrite = 0;  // Temporary variable

    // Setup the accelerometer
    dataToWrite = 0;

    //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;


    // Step 1: Configure ODR-26Hz and FS-2g
    errorAccumulator += IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

    // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
    if (clearStep) {
        errorAccumulator += IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
    } else {
        errorAccumulator += IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
    }

    // Step 3:	Enable pedometer algorithm
    errorAccumulator += IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

    //Step 4:	Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
    errorAccumulator += IMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

    return errorAccumulator;
}

