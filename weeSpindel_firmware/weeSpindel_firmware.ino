// Stripped down version of the iSpindel logic, using ESP-NOW to
// transmit data packets. The data is formatted as a MQTT-style JSON
// message, with a "topic payload" structure. This allows a ESP-NOW
// gateway to bounce it straight to a MQTT broker.
//
// Note that this drops the DS18B20. The MPU sensor seems like it's in the
// same ballpark (usually under .5C difference), although it is
// a bit slower at catching temperature changes.

extern "C" {
  #include <espnow.h>
  #include <user_interface.h>
}
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// this isn't important unless you're trying to make a completely
// wireless bridge work.
#define WIFI_CHANNEL 7

// Time to linger after a transmit. In practice, we should never
// actually timeout since we'll sleep as soon as the transmit
// completes.
#define SEND_TIMEOUT 50

// Maximum time to be awake, in ms. This is needed in case the MPU sensor
// fails to return any samples.
#define WAKE_TIMEOUT  250

// I2C pins
#define SDA_PIN 4
#define SCL_PIN 5

// pull low to run in calibration mode
#define CALIBRATE_PIN 14

// number of tilt samples to average
#define MAX_SAMPLES 5

// Normal interval should be long enough to stretch out battery life. Since
// we're using the MPU temp sensor, we're probably going to see slower
// response times so longer intervals aren't a terrible idea.
#define NORMAL_INTERVAL 1800

// In calibration mode, we need more frequent updates.
#define CALIBRATION_INTERVAL 30

// This prefixes the identifier in the JSON message, which
// I feed directly to MQTT.
#define TOPIC_PREFIX "sensors/"

// When the battery cell (default assumes NiMH) gets this low,
// the ESP switches to every
// LOW_VOLTAGE_MULTIPLIER*SLEEP_UPDATE_INTERVAL second updates. Hopefully
// someone is monitoring that stuff and can swap batteries before it really dies.
#define LOW_VOLTAGE_THRESHOLD 1.2
#define LOW_VOLTAGE_MULTIPLIER 4

// when we booted
static unsigned long started = 0;

// This eliminates the intense startup spike caused by RF calibration.
// Without it, booting from an AAA NiMH is unreliable unless it's fresh
// off the charger. Normal deep sleeps have RFCAL disabled, so that hasn't been
// a huge problem.
RF_PRE_INIT() {
  started = millis();

  // Note: we don't need the radio off entirely because we're going to need it
  // pretty soonish.
  system_phy_set_powerup_option(2);  // stop the RFCAL at boot
  wifi_set_opmode_current(NULL_MODE);  // set Wi-Fi to unconfigured, don't save to flash

  // this *shouldn't* be needed, but let's try it.
  wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
}

//RF_MODE(RF_NO_CAL)  // maybe equivalent?

//------------------------------------------------------------
static String nodeid;
static MPU6050 mpu;
static long sleep_interval = NORMAL_INTERVAL;

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
static void actuallySleep() {
  // If we haven't already done this...
  mpu.setSleepEnabled(true);
  
  double uptime = (millis() - started)/1000.;
  
  long willsleep = sleep_interval - uptime;
  if( willsleep <= sleep_interval/2) {
    // If we somehow ended up awake longer than half a sleep interval,
    // sleep longer. This shouldn't happen in practice.
    willsleep = sleep_interval;
  }
  Serial.printf("Deep sleeping %d seconds after %.3g awake\n", willsleep, uptime);

  ledOff();
  ESP.deepSleepInstant(willsleep * 1000000, WAKE_NO_RFCAL);
}

//-----------------------------------------------------------------
static double voltage = HUGE_VAL;

static inline double readVoltage() {
  // 4.03v to 1.0v voltage divider (1M -> 330k 1%)
  // NOTE: overkill for an NiMH cell, but the weeSpindel
  // has a 10440 Li-ion option too.
  return (voltage = map(analogRead(A0),0,1024,0,4000)/1000.);
}

//--------------------------------------------------------------
static unsigned nsamples = 0;
static float samples[MAX_SAMPLES];
static float temperature = 0.0;
static unsigned long sent = 0;  // time of transmission

float round1(float value) {
   return (int)(value * 10 + 0.5) / 10.0;
}

static void sendSensorData() {
  Serial.println("Sending data...");

  // NOTE: max message length is 250 bytes.
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonArray& s = root.createNestedArray("samples");

  // calculate average. Median might
  // throw away initial "bad" readings.
  double sum = 0;
  for( int i = 0; i < nsamples; i ++ ) {
    sum += samples[i];
    // throw the samples into the message too 
    s.add(round1(samples[i]));
  }

  root["tilt"] = round1(sum / nsamples);
  root["tt"] = round1(temperature);
  root["v"] = voltage;
  root["interval"] = sleep_interval;

  String jstr;
  root.printTo(jstr);

  String msg = TOPIC_PREFIX;
  msg += nodeid;
  msg += " ";
  msg += jstr;

  Serial.println(msg);

  // Bring up the network as late as possible
  Serial.println("Initialize network");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if( esp_now_init() != 0 ) {
    Serial.println("ESPNOW init failed");
    actuallySleep();
  }

  // Configure for broadcast.
  static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(broadcast_mac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

  // We're doing a broadcast so we won't get an ACK packet, but this will
  // tell us when it's safe to go to sleep, which shaves most of SEND_TIMEOUT
  // off our wake cycle.
  esp_now_register_send_cb([](uint8_t *mac, uint8_t Status)
  {
    Serial.println("send status " + String(Status));

    // this should force us to sleep almost immediately when the loop()
    // next rolls around.
    sent = 1;
  });

  esp_now_send(NULL, (u8*)msg.c_str(), msg.length()+1);
}

void setup() {
  pinMode(led, OUTPUT);
  ledOn();

  {
    char buf[12];
    nodeid = "weeSpindel-";
    nodeid += itoa(ESP.getChipId(),buf,16);
  }

  Serial.begin(115200);
  Serial.println("Reboot");

  Serial.print("Node id: ");
  Serial.println(nodeid);
  
  Serial.print("Booting because ");
  Serial.println(ESP.getResetReason());

  Serial.println("Build: " __DATE__ " " __TIME__ );
  
  pinMode(CALIBRATE_PIN,INPUT_PULLUP);
  if( digitalRead(CALIBRATE_PIN) ) {
    Serial.println("Normal mode");

    readVoltage();
    bool lowv = !(voltage != HUGE_VAL && voltage > LOW_VOLTAGE_THRESHOLD);
    if( lowv ) {
      Serial.println("Voltage below threshold, sleeping longer");
      sleep_interval *= LOW_VOLTAGE_MULTIPLIER;
    }
  } else {
    Serial.println("Calibration mode");

    // The only difference between "normal" and "calibration"
    // is the update frequency. We still deep sleep between samples.
    sleep_interval = CALIBRATION_INTERVAL;
  }

  Serial.println("Starting MPU-6050");
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
  
  Serial.println("Finished setup");
}

static float calculateTilt(float ax, float az, float ay ) {
  float pitch = (atan2(ay, sqrt(ax * ax + az * az))) * 180.0 / M_PI;
  float roll = (atan2(ax, sqrt(ay * ay + az * az))) * 180.0 / M_PI;
  return sqrt(pitch * pitch + roll * roll);
}

void loop() {
  if( sent ) {
    if( millis()-sent > SEND_TIMEOUT ) {
      actuallySleep();
    }
  } else if( nsamples >= MAX_SAMPLES || (millis()-started) > WAKE_TIMEOUT ) {
    sent = millis();
    sendSensorData();
  } else if( nsamples < MAX_SAMPLES && mpu.getIntDataReadyStatus() ) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &az, &ay);

    float tilt = calculateTilt(ax, az, ay);
    if( tilt > 0.0 ) {
      // we sometimes get bogus zero initial readings
      // after a hard boot. Ignore them.
      samples[nsamples++] = tilt;
    }

    if( nsamples >= MAX_SAMPLES ) {
      // As soon as we have all our samples, read the temperature
      temperature = mpu.getTemperature() / 340.0 + 36.53;

      // ... and put the MPU back to sleep. No reason for it to
      // be sampling while we're doing networky things.
      mpu.setSleepEnabled(true);
    }
  }

  // mpu.getIntDataReadyStatus() hits the I2C bus. We don't need to poll every ms.
  delay(5);
}
