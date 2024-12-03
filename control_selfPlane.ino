/*
CONTROL PID
ANTIWINDUP
FILTRO PASA BAJAS (EN DERIVADOR) - FEEDBACK LOOP CON INTEGRADOR EN EL FEEDBACK
*/

#include <LoRa.h>
#include <SPI.h>
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP085.h>
#include "MPU9250.h"
#include "BluetoothSerial.h"

#define IS_BLUETOOTH 1
#define DEBUG 1
#if DEBUG == 1
  #define debug(x) {Serial.print(x); delay(10);}
  #define debugln(x) {Serial.println(x); delay(10);}
#else
  #define debug(x)
  #define debugln(x)
#endif

#define BAUDRATE 115200
#define BAUDRATE_GPS 9600
#define PI 3.141592
#define LORA_FREQ 433E6

#define LORA_SCK 32
#define LORA_MISO 33
#define LORA_MOSI 25
#define LORA_CS 26
#define LORA_RST 27
#define LORA_DI0 4

#define ECHO_PIN 23
#define TRIGGER_PIN 22
#define SV_PITCH_PIN 21
#define SV_ROLL_PIN 19
#define SDA_PIN 17
#define SCL_PIN 16
// #define BUZ_PIN 4
#define LED_PIN 2

/* ----- CONSTANT VARIABLES ----- */
struct cVar {
  /* Ultrasonic sensor */
  static constexpr uint8_t TEMPERATURE_LOCAL = 20;

  /* Final approach with ultrasonic sensor */
  static constexpr uint8_t PITCH_MAX = 30;
  static constexpr uint16_t ALTITUDE_THRESHOLD = 40;    // NEEDS TO BE ADJUSTED

  /* Turn control */
  static constexpr uint8_t TURN_BANK_MAX = 20;
  static constexpr uint8_t TURN_PITCH_MAX = 15;

  /* Attitude control */
  static constexpr uint8_t SV_CENTER_ROLL = 90;
  static constexpr uint8_t SV_CENTER_PITCH = 92;
  // static constexpr uint16_t ALTITUDE_REF = 2000;
};

struct kVar {
  static constexpr float K_APPROACH_PITCH = 3.5;
  static constexpr float K_BANK_YAW = 1.5;
  static constexpr float K_BANK_PITCH = 2.0;
  static constexpr float K_ROLL = 2.5;
  static constexpr float K_PITCH = 2.0;
};

// /* Landing point */
// struct varLanding {
//   static int32_t longitude;
//   static int32_t latitude;
//   static float altitude;
// };

/* Bluetooth */
constexpr uint16_t timeInterval = 100;
constexpr char *ESP_NAME = "YoESP32";     // ESP32 Bluetooth name

/* Mission control */
/* 0 = Stabilizer (default), 1 = Turning, 2 = Approach*/
uint8_t isMission = 0;         // Whats my mission?
bool isArmed = true;             // Am i armed?

/* Turn control */
/* 0 = Stabilizer, 1 = Right, 2 = Left */
uint8_t isTurn = 0;            // Where am i turning to?
uint16_t YAW_SETPOINT = 90;
uint8_t YAW_TOLERANCE = 30;

/* Attitude control */
// float controlInput = 0.0, controlOutput, controlSetPoint = 90;
// float Kp = 1; Ki = 0.0001; Kd = 0.0001;
// PID rollPID(&controlInput, &controlOutput, &controlSetPoint, Kp, Ki, Kd, DIRECT);

Adafruit_BMP085 BMP;
MPU9250 MPU;
Servo ServoRoll;
Servo ServoPitch;
BluetoothSerial BT_ESP;



void setup() {
  /* ----- INIT ----- */
  Serial.begin(BAUDRATE);
  BT_ESP.begin(ESP_NAME);
  Wire.begin(SDA_PIN, SCL_PIN); delay(10);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  /* LED pin for debugging */
  pinMode(LED_PIN, OUTPUT);

  /* Ultrasonic sensor */
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);

  /* PID control */
  // rollPID.SetMode(AUTOMATIC);

  /* ----- MODULES INIT ----- */
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DI0); delay(10);
  if (!LoRa.begin(LORA_FREQ)) {
    debugln("LoRa init failed");
    while (1) {
      ledError();
    }
  } else {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(500E3); delay(10);
  }

  if (!MPU.setup(0x68)) {
    debugln("MPU9250 init failed");
    while (1) {
      ledError();
    }
  }

  // if (!BMP.begin()) {
  //   debugln("BMP180 init failed");
  //   while (1) {
  //     ledError();
  //   }
  // }


  /* Timers for ESP32 pwm allocation */
  debugln("Configurating servos and ultrasonic...");
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  /* Adjusting servos */
  ServoRoll.setPeriodHertz(50);
  ServoRoll.attach(SV_ROLL_PIN, 1000, 2000);
  ServoRoll.write(cVar::SV_CENTER_ROLL);

  ServoPitch.setPeriodHertz(50);
  ServoPitch.attach(SV_PITCH_PIN, 1000, 2000);
  ServoPitch.write(cVar::SV_CENTER_ROLL);

  
  #if IS_BLUETOOTH == 1
    delay(5000);
    BT_ESP.println("Enter desired yaw: ");
    while (true) {
      if (BT_ESP.available() > 0) {
        int _dataSerial = BT_ESP.parseInt();
        if (BT_ESP.read() == '\r') {}   // To stop blocking (just numbers)

        if (_dataSerial >= 0 && _dataSerial < 360) {
          YAW_SETPOINT = abs(_dataSerial);
          BT_ESP.println("Setpoint yaw set to: " + String(YAW_SETPOINT));
          break;
        } else {
          BT_ESP.println("Invalid input");
        }
      }
    }

    delay(1000);
    BT_ESP.println("Enter yaw tolerance: ");
    while (true) {
      if (BT_ESP.available() > 0) {
        int _dataSerial = BT_ESP.parseInt();
        if (BT_ESP.read() == '\r') {}   // To stop blocking (just numbers)

        if (_dataSerial >= 5 && _dataSerial < 40) {
          YAW_TOLERANCE = abs(_dataSerial);
          BT_ESP.println("Yaw tolerance set to: " + String(YAW_TOLERANCE));
          break;
        } else {
          BT_ESP.println("Invalid input");
        }
      }
    }
  #endif

  delay(1000);
}


void ledError() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}


void calibration() {
  digitalWrite(LED_PIN, HIGH);
  BT_ESP.println("Calibrating ...");
  
  MPU.verbose(true);
  delay(2000);

  MPU.calibrateAccelGyro();
  BT_ESP.println("Accelerometer calibrated");
  delay(2000);

  /* LED signal to start magnetometer manual calibration */
  ledError();
  ledError();
  ledError();
  digitalWrite(LED_PIN, HIGH);

  BT_ESP.println("Wave device in an figure eight");
  MPU.calibrateMag();
  BT_ESP.println("Magnetometer calibrated");
  delay(2000);

  MPU.verbose(false);
  digitalWrite(LED_PIN, LOW);
}


float HCSR04_Measure() {
  constexpr float SPEED_SOUND = sqrt(1.4 * 286 * (273.15 + cVar::TEMPERATURE_LOCAL));
  constexpr float SPEED_CONVERSION = (SPEED_SOUND * (100.0/1.0) * (1.0/1000000.0)) / 2;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  uint16_t durationPulse = pulseIn(ECHO_PIN, HIGH);
  float distanceMeasured = durationPulse * SPEED_CONVERSION;

  return distanceMeasured;
}


void stabilizer(int _roll, int _pitch, float kRoll = 1.0, float kPitch = 1.0, int angleRoll = 0, int anglePitch = 0) {
  ServoRoll.write(((_roll - angleRoll) * kRoll * -1) + cVar::SV_CENTER_ROLL);
  ServoPitch.write(((_pitch - anglePitch) * kPitch * -1) + cVar::SV_CENTER_PITCH);
}


/* Turning control */
void controlTurning(int _roll, int _pitch, int _yaw_turn) {
  /* 
    _yaw_turn_bank: positive values means that the airplane must turn to the right,
    a negative value means left turn.
  */
  int16_t _yaw_turn_bank = (_yaw_turn < -180) ? (_yaw_turn + 360) : _yaw_turn;
  isTurn = (_yaw_turn_bank < 0) ? 1 : 2;      // 0 = Stabilizer, 1 = Right, 2 = Left
  Serial.print(isTurn);
  Serial.print("\t");
  Serial.println(_yaw_turn_bank);

  if (isTurn == 1) {
    stabilizer(_roll, _pitch, kVar::K_BANK_YAW, kVar::K_BANK_PITCH, cVar::TURN_BANK_MAX * -1, cVar::TURN_PITCH_MAX);
  } else {
    stabilizer(_roll, _pitch, kVar::K_BANK_YAW, kVar::K_BANK_PITCH, cVar::TURN_BANK_MAX, cVar::TURN_PITCH_MAX);
  }
}


/* Final approach pitch control */
void controlApproach(int _roll, float _distanceGround) {
  int8_t pitchControlled = ((float)(cVar::PITCH_MAX * kVar::K_APPROACH_PITCH) / cVar::ALTITUDE_THRESHOLD) * (cVar::ALTITUDE_THRESHOLD - _distanceGround);

  ServoRoll.write((_roll * kVar::K_ROLL * -1) + cVar::SV_CENTER_ROLL);
  ServoPitch.write(pitchControlled + cVar::SV_CENTER_PITCH);
} 

/* -------------------------------------------------- */

void loop() {
  if (MPU.update()) {
    int16_t roll, pitch, yaw;
    static uint32_t timePrevious = millis();

    if (millis() > timePrevious + 25) {
      /* Quaternions use and calculation */
      float qw = MPU.getQuaternionW(), qx = MPU.getQuaternionX(), qy = MPU.getQuaternionY(), qz = MPU.getQuaternionZ();
      float _roll_atan1 = 2 * ((qw*qx) + (qy*qz));
      float _roll_atan2 = 1 - 2 * ((qx*qx) + (qy*qy));
      float _yaw_atan1 = 2 * ((qw*qz) + (qx*qy));
      float _yaw_atan2 = 1 - 2 * ((qy*qy) + (qz*qz));

      /* Library functions calculated with quaternions */
      // roll = MPU.getRoll();
      // pitch = MPU.getPitch();
      // yaw = MPU.getYaw();
      
      roll = atan2(_roll_atan1, _roll_atan2) * 180/PI;
      pitch = asin(2 * ((qw*qy) - (qx*qz))) * 180/PI;
      yaw = atan2(_yaw_atan1, _yaw_atan2) * 180/PI;
      yaw = (yaw < 0) ? (yaw + 360) : yaw;

      int16_t yaw_turn = YAW_SETPOINT - yaw;
      float distanceGround = HCSR04_Measure();

      /* ----- MISSIONS ----- */
      /* 0 = Stabilizer, 1 = Turning, 2 = Approaching */
      isMission = (abs(yaw_turn) > YAW_TOLERANCE) ? 1 : 0;     // Turning detection
      isMission = (distanceGround < cVar::ALTITUDE_THRESHOLD) ? 2 : isMission;    // Approach mission is prioritized over turning


      if (isArmed) {
        switch (isMission) {
          case 1:
            controlTurning(roll, pitch, yaw_turn);
            digitalWrite(LED_PIN, LOW);
            break;

          case 2:
            controlApproach(roll, distanceGround);
            digitalWrite(LED_PIN, LOW);
            break;

          default:
            // controlInput = roll;
            // rollPID.compute();
            // stabilizer(controlOutput, pitch);

            stabilizer(roll, pitch, kVar::K_ROLL, kVar::K_PITCH);
            digitalWrite(LED_PIN, HIGH);
            isTurn = 0;
            break;
        }
      }


      /* ----- DEBUGGING ----- */
      // debug(roll); debug(",");
      // debug(pitch); debug(",");
      // debug(yaw); debug(",");
      // debug(distanceGround); debug(",");
      // debugln(isMission);

      // controlTurning(roll, yaw_turn);
      // controlApproach(roll, distanceGround);

      // ServoRoll.write((roll * kVar::K_ROLL * -1) + cVar::SV_CENTER_ROLL);
      // ServoPitch.write((pitch * kVar::K_PITCH * -1) + cVar::SV_CENTER_PITCH);


      // float temperature = BMP.readTemperature();
      // float altitude = BMP.readAltitude();
      
      String dataPacket = String(roll) + String(",") + String(pitch) + String(",") + String(yaw) + String(",") + 
                          String(distanceGround) + String(",") + String(isTurn) + String(",") + String(isMission); 


      /* ----- COMMUNICATION ----- */
      // Serial.println(dataPacket);

      #if IS_BLUETOOTH == 1
        static uint32_t timePrevious_BT = millis();
        if (millis() > timePrevious_BT + timeInterval) {
          BT_ESP.println(dataPacket);
          timePrevious_BT = millis();
        }

        if (BT_ESP.available() > 0) {
          int _dataSerial = BT_ESP.parseInt();
          if (BT_ESP.read() == '\r') {}   // To stop blocking (just numbers)
          
          switch (_dataSerial) {
            case 10:
              calibration();
              break;

            case 20:
              isArmed = false;
              break;

            case 21:
              isArmed = true;
              break;
          }
        }
      #endif

      /* Sending data */
      LoRa.beginPacket();
      LoRa.print(dataPacket);
      LoRa.endPacket();

      timePrevious = millis();
    }
  }
}