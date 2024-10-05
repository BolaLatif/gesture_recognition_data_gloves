#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <BluetoothSerial.h>
#include <Adafruit_TestBed.h>

//Global Variables
unsigned long start_comm = 0;
unsigned long end_comm = 0;
unsigned long one_chanell_time = 0;
unsigned long start_comm_chanells = 0;
unsigned long end_comm_chanells = 0;
unsigned long all_chanells_time = 0;
unsigned long timecheck_1s = 0;
unsigned long timecheck_start = 0;
uint8_t channell = 0;
uint8_t frame_num = 0;
extern Adafruit_TestBed TB;

const char *pin = "1234";  // Change this to a more secure PIN.
String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define SDA 22
#define SCL 20
#define I2C_Freq 100000
#define NEOPIXEL_PIN 0
#define VBATPIN A13

const int powerLatch = 19;

//Objects
QWIICMUX Mux;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
BluetoothSerial SerialBT;

void setup() {
  pinMode(powerLatch, OUTPUT);
  digitalWrite(powerLatch, LOW);

  // Setup the RGB light as a Battery indicator
  TB.neopixelPin = NEOPIXEL_PIN;
  TB.neopixelNum = 1;
  TB.begin();

  //Setup Serial Communication
  Serial.begin(115200);
  delay(10);
  Wire.begin(SDA, SCL, I2C_Freq);
  delay(100);

  //Bluetooth Initializing and Set Name for Bluetooth
  SerialBT.begin(device_name);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
#ifdef USE_PIN
  SerialBT.setPin(pin);
  Serial.println("Using PIN");
#endif
  delay(2000);

  //MUX & BNO Setup

  if (Mux.begin(0x71, Wire)) {
    Serial.println("TCA9548A OK");
    Mux.disablePort(0);
    Mux.disablePort(7);
  } else {
    Serial.println("TCA9548A Fehler");
    while (1)
      ;
  }

  for (int i = 1; i < 7; i++) {
    Mux.setPort(i);
    bno.begin(OPERATION_MODE_COMPASS);
    bno.setExtCrystalUse(false);
    int32_t mode = 0;
    if (!bno.begin()) {
      Serial.println("BNO055 not detected!");
    }
    if (bno.begin()) {
      Serial.println("BNO055 is connected!");
      bno.setMode(OPERATION_MODE_COMPASS);
      mode = bno.getMode();
      Serial.println(mode);
    }
  }
}

void loop() {
  frame_num += 1;
  if (frame_num == 1) {
    timecheck_start = millis();
  }
  start_comm_chanells = millis();
  String dataToSend = "{\"frameData\":[";
  for (int i = 1; i < 7; i++) {
    start_comm = millis();
    Mux.setPort(i);
    sensors_event_t event;
    bno.getEvent(&event);
    imu::Quaternion quat = bno.getQuat();

    dataToSend = dataToSend + "[" + quat.w() + "," + quat.x() + "," + quat.y() + "," + quat.z() + "]";
    if (i == 6) {
      dataToSend = dataToSend + ",[0.0,0.0,0.0,0.0]]";
    } else {
      dataToSend = dataToSend + ",";
    }
    end_comm = millis();
    one_chanell_time = (end_comm - start_comm);
  }
  end_comm_chanells = millis();
  all_chanells_time = (end_comm_chanells - start_comm_chanells);
  if ((millis() - timecheck_start) >= 1000) {

    // Programming RGB lamp as indicator for Battery (Green, Orange, Red)

    float measuredvbat = analogReadMilliVolts(VBATPIN);
    measuredvbat *= 2;    // is divided by 2, so multiply back
    measuredvbat /= 1000; // convert to volts!
    if (measuredvbat >= 3.50){TB.setColor(0x00FF00);}
    if (measuredvbat <= 3.50 && measuredvbat >= 3.40){TB.setColor(0xFF8000);}
    if (measuredvbat <= 3.40){TB.setColor(0xFF0000);}
    dataToSend = dataToSend + ",\"battery\":" + measuredvbat;
    frame_num = 0;
  }
  dataToSend = dataToSend + "}";
  SerialBT.println(dataToSend);

  delay(10);

}
