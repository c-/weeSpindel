// Stripped down version of the iSpindel logic, using ESP-NOW to
// transmit data packets. The data is formatted as a MQTT-style JSON
// message, with a "topic payload" structure. This allows a ESP-NOW
// gateway to bounce it straight to a MQTT broker.

extern "C" {
  #include <espnow.h>
}
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


#define WIFI_CHANNEL 7
#define SEND_TIMEOUT 100
#define DS18B20_PIN 12
#define SDA_PIN 4
#define SCL_PIN 5
#define INT_PIN 15 // not used currently
#define CALIBRATE_PIN 14  // pull low to run in calibration mode
#define DS18RESOLUTION 10
#define MAX_SAMPLES 20
#define SLEEP_UPDATE_INTERVAL 600
#define TOPIC_PREFIX "sensors/"

// When the battery cell (default assumes NiMH) gets this low,
// the ESP switches to every 3*SLEEP_UPDATE_INTERVAL updates. Hopefully
// someone is monitoring that stuff and can swap batteries before it really dies.
#define LOW_VOLTAGE_THRESHOLD 1.0

//------------------------------------------------------------
static String nodeid;
static OneWire oneWire(DS18B20_PIN);
static DallasTemperature ds18(&oneWire);
static MPU6050 mpu;

//------------------------------------------------------------
static const int led = LED_BUILTIN;

static inline void ledOn() {
  // reduce the brightness a whole bunch
  analogWrite(led, PWMRANGE-20);
}
static inline void ledOff() {
  analogWrite(led,0);
  digitalWrite(led, HIGH);
}

//-----------------------------------------------------------------
static double voltage = HUGE_VAL;
static unsigned long started = 0;

static void actuallySleep(void) {
  bool lowv = !(voltage != HUGE_VAL && voltage > LOW_VOLTAGE_THRESHOLD);
  
  double uptime = (millis() - started)/1000.;
  
  long willsleep = SLEEP_UPDATE_INTERVAL * (lowv ? 3 : 1) - uptime;
  if( willsleep <= SLEEP_UPDATE_INTERVAL/2) {
    willsleep = SLEEP_UPDATE_INTERVAL;
  }
  Serial.printf("Deep sleeping %d seconds after %.2g awake\n", willsleep, uptime);

  ledOff();
  ESP.deepSleepInstant(willsleep * 1000000, WAKE_NO_RFCAL);
}

//-----------------------------------------------------------------
static inline double readVoltage() {
  // 4.03v to 1.0v voltage divider (1M -> 330k)
  // NOTE: overkill for an NiMH cell, but I use the same circuit
  // for a single AA right up to 18650 cells.
  return (voltage = map(analogRead(A0),0,1024,0,4000)/1000.);
}

//--------------------------------------------------------------
static unsigned nsamples = 0;
static float samples[MAX_SAMPLES];

static void sendSensorData() {
  ledOn();
  
  Serial.println("Sending update...");
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  // calculate average
  double sum = 0;
  for( int i = 0; i < nsamples; i ++ ) {
    sum += samples[i]; 
  }

  root["tt"] = ds18.getTempCByIndex(0);
  root["tilt"] = sum / nsamples;
  root["nsamples"] = nsamples;  // remove in final version
  root["v"] = readVoltage();
  root["id"] = nodeid;

  String jstr;
  root.printTo(jstr);

  String msg = TOPIC_PREFIX;
  msg += nodeid;
  msg += "/status";
  msg += " ";
  msg += jstr;
  Serial.println(msg);
  esp_now_send(NULL, (u8*)msg.c_str(), msg.length()+1);

  Serial.println("Finished update...");
  ledOff();
}

static unsigned long dsready = 0;

static void startTempReading() {
  ds18.requestTemperatures();
  dsready = millis() + (750/(1<<(12-DS18RESOLUTION)));
}
static boolean isTempReady() {
  return millis() >= dsready;
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
static void dmpDataReady() {
    mpuInterrupt = true;
}

static boolean normal_mode = false;
void setup() {
  started = millis();

  {
    char buf[16];
    nodeid = "weeSpindel-";
    nodeid += itoa(ESP.getChipId(),buf,16);
  }

  pinMode(led, OUTPUT);
  ledOn();

  Serial.begin(115200);
  Serial.println("Reboot");

  Serial.print("Node id: ");
  Serial.println(nodeid);
  
  Serial.print("Booting because ");
  Serial.println(ESP.getResetReason());

  Serial.println("Build: " __DATE__ " " __TIME__ );
  
  pinMode(CALIBRATE_PIN,INPUT_PULLUP);
  normal_mode = digitalRead(CALIBRATE_PIN);
  Serial.println(normal_mode ? "Normal mode" : "Calibration mode");

  Serial.println("Initialize network");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if( esp_now_init() != 0 ) {
    Serial.println("ESPNOW init failed");
    actuallySleep();
  }

  // Configure for broadcast
  static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(broadcast_mac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

  // kick off the temperature conversion
  Serial.println("Starting Temperature Reading");
  ds18.begin();
  DeviceAddress tempDeviceAddress;
  ds18.getAddress(tempDeviceAddress, 0);
  ds18.setResolution(tempDeviceAddress, DS18RESOLUTION);
  ds18.setWaitForConversion(false);
  startTempReading();

  Serial.println("Starting MPU-6050 Reading");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize();
  //pinMode(INT_PIN, INPUT);
  //devStatus = mpu.dmpInitialize();
  //mpu.setDMPEnabled(true);
  //attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  mpu.setTempSensorEnabled(true);
  mpu.setInterruptLatch(0); // pulse
  mpu.setInterruptMode(1);  // Active Low
  mpu.setInterruptDrive(1); // Open drain
  mpu.setRate(17);
  mpu.setIntDataReadyEnabled(true);
  
  Serial.println("Ready");
}

static unsigned long sent = 0;

static float calculateTilt(float ax, float az, float ay ) {
  float pitch = (atan2(ay, sqrt(ax * ax + az * az))) * 180.0 / M_PI;
  float roll = (atan2(ax, sqrt(ay * ay + az * az))) * 180.0 / M_PI;
  return sqrt(pitch * pitch + roll * roll);
}

void loop() {
  if( sent==0 && isTempReady() ) {
    sendSensorData();
    sent = millis();

    if( !normal_mode ) {
      // in calibration mode, just keep cycling
      nsamples = 0;
      startTempReading();
      dsready += 1000;  // but less often
      sent = 0;
    }
  } else if( normal_mode && sent ) {
    // FIXME: this could be reduced if we checked for ACK's
    if (millis()-sent > SEND_TIMEOUT) {
      actuallySleep();
    }
  } else if( nsamples < MAX_SAMPLES && mpu.getIntDataReadyStatus() ) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &az, &ay);
    samples[nsamples++] = calculateTilt(ax, az, ay);
  }
  delay(1);
}
