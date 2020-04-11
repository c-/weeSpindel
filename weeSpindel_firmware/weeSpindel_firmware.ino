// Stripped down version of the iSpindel logic, using ESP-NOW to
// transmit data packets. The data is formatted as a MQTT-style JSON
// message, with a "topic payload" structure. This allows a ESP-NOW
// gateway to bounce it straight to a MQTT broker.
//
// Note that this drop the DS18B20. The MPU sensor seems like it's in the
// same ballpark (usually under .5C difference), although it is
// a bit slower at catching temperature changes.

extern "C" {
  #include <espnow.h>
}
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define WIFI_CHANNEL 7
#define SEND_TIMEOUT 100
#define SDA_PIN 4
#define SCL_PIN 5
#define CALIBRATE_PIN 14  // pull low to run in calibration mode
#define MAX_SAMPLES 5   // number of tilt samples to average
#define SLEEP_UPDATE_INTERVAL 600
#define CAL_DELAY 30 * 1000  // send a reading every 30 seconds in calibration mode
#define TOPIC_PREFIX "sensors/"

// When the battery cell (default assumes NiMH) gets this low,
// the ESP switches to every 3*SLEEP_UPDATE_INTERVAL updates. Hopefully
// someone is monitoring that stuff and can swap batteries before it really dies.
#define LOW_VOLTAGE_THRESHOLD 1.0

//------------------------------------------------------------
static String nodeid;
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

  // turn off the sensor
  mpu.setSleepEnabled(true);
  
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
  
  Serial.println("Sending data...");
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  // calculate average
  double sum = 0;
  for( int i = 0; i < nsamples; i ++ ) {
    sum += samples[i]; 
  }

  root["tilt"] = sum / nsamples;
  root["tt"] = mpu.getTemperature() / 340.0 + 36.53;
  root["v"] = readVoltage();

  String jstr;
  root.printTo(jstr);

  String msg = TOPIC_PREFIX;
  msg += nodeid;
  msg += " ";
  msg += jstr;
  esp_now_send(NULL, (u8*)msg.c_str(), msg.length()+1);
  Serial.println(msg);

  ledOff();
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

  Serial.println("Starting MPU-6050 Reading");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize();
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
  if( nsamples >= MAX_SAMPLES ) {
    sendSensorData();
    sent = millis();

    // in calibration mode, just keep cycling after a delay
    if( !normal_mode ) {
      sent += CAL_DELAY;
      nsamples = 0;
    }
  } else if( normal_mode && sent && millis()-sent > SEND_TIMEOUT ) {
    actuallySleep();
  } else if( millis() > sent && nsamples < MAX_SAMPLES && mpu.getIntDataReadyStatus() ) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &az, &ay);
    samples[nsamples++] = calculateTilt(ax, az, ay);
  }
  delay(1);
}
