#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP085.h>
#include "MPU9250.h"

#define IS_BLUETOOTH 0
#define DEBUG 0

#if IS_BLUETOOTH == 1
  #include "BluetoothSerial.h"

  /* Bluetooth */
  constexpr uint16_t timeInterval = 100;
  constexpr char *ESP_NAME = "YoESP32";     // ESP32 Bluetooth name
  
  BluetoothSerial BT_ESP;
#endif
#if DEBUG == 1
  #define debug(x) {Serial.print(x); delay(10);}
  #define debugln(x) {Serial.println(x); delay(10);}
#else
  #define debug(x)
  #define debugln(x)
#endif

#define BAUDRATE 115200
#define LORA_FREQ 433E6

#define LORA_SCK 32
#define LORA_MISO 33
#define LORA_MOSI 25
#define LORA_CS 26
#define LORA_RST 27
#define LORA_DI0 4

#define PIN_VOLTAGE 23
#define PIN_CURRENT 22
#define PIN_SV_PITCH 21
#define PIN_SV_ROLL 19
#define PIN_SDA 17
#define PIN_SCL 16
#define PIN_BUZ 15
#define PIN_LED 2

/* ----- CONSTANT VARIABLES ----- */
struct cVar {
  /* BMP sensor */
  static constexpr uint16_t BMP_CALIBRATION_VALUES = 50;

  /* Final approach with ultrasonic sensor */
  static constexpr uint8_t PITCH_MAX = 30;
  static constexpr uint16_t ALTITUDE_THRESHOLD = 68;

  /* Turn control */
  static constexpr uint8_t TURN_BANK_MAX = 30;
  static constexpr uint8_t TURN_PITCH_MAX = 15;

  /* Attitude control */
  static constexpr uint8_t SV_CENTER = 90;
  // static constexpr uint16_t ALTITUDE_REF = 2000;
};

struct PIDvar {
  static constexpr uint16_t SETPOINT = 100;
  static constexpr float INTEGRAL_WINDUP_MIN = -50;
  static constexpr float INTEGRAL_WINDUP_MAX = 50;
  static constexpr float DERIVATIVE_LPF_ALPHA = 0.1;
}; 

/* PID parameters */
float input = 0.0;
float output = 0.0;
float integral = 0.0;
float errorLast = 0.0;
unsigned long timeLast = 0;

/* Order is Kp, Ki, Kd */
float PID_STABROLL_VAR[3] = {1.88, 0.1, 0.005};
float PID_STABPITCH_VAR[3] = {1.8, 0.5, 0.008};
float PID_TURNROLL_VAR[3] = {1.88, 0.1, 0.005};
float PID_TURNPITCH_VAR[3] = {1.8, 0.5, 0.008};

/* Mission control */
/* 0 = Stabilizer (default), 1 = Turning, 2 = Approach*/
uint8_t isMission = 0;         // Whats my mission?
bool isArmed = true;             // Am i armed?

/* Turn control */
/* 0 = Stabilizer, 1 = Right, 2 = Left */
uint8_t isTurn = 0;            // Where am i turning to?
uint16_t YAW_SETPOINT = 110;
uint8_t YAW_TOLERANCE = 30;

/* Other variables */
float ALTITUDE_DEFINED;

Servo ServoRoll;
Servo ServoPitch;
Adafruit_BMP085 BMP;
MPU9250 MPU;



void ledError(int time = 100) {
  digitalWrite(PIN_LED, HIGH);
  delay(time);
  digitalWrite(PIN_LED, LOW);
  delay(time);
}


void setup() {
  /* ----- INIT ----- */
  Serial.begin(BAUDRATE);
  Wire.begin(PIN_SDA, PIN_SCL); delay(10);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  /* LED pin for debugging */
  pinMode(PIN_LED, OUTPUT);

  /* Current and voltage sensors */
  // pinMode(PIN_VOLTAGE, INPUT);
  // pinMode(PIN_CURRENT, INPUT);

  /* Buzzer */
  // pinMode(PIN_BUZ, OUTPUT);

  /* ----- MODULES INIT ----- */
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DI0); delay(10);

  if (!MPU.setup(0x68)) {
    debugln("MPU9250 init failed");
    while (1) {
      ledError(1000);
    }
  }

  if (!BMP.begin()) {
    debugln("BMP180 init failed");
    while (1) {
      ledError(2000);
    }
  } else {
    for (int i = 0; i < cVar::BMP_CALIBRATION_VALUES; i++) {
      ledError();
      ALTITUDE_DEFINED += BMP.readAltitude();
    }
    ALTITUDE_DEFINED /= cVar::BMP_CALIBRATION_VALUES;
  }

  ledError(500);
  ledError(500);
  ledError(500);
  calibration();    // Accel and magnetomer calibration
  
  /* Timers for ESP32 pwm allocation */
  debugln("Configurating servos and ultrasonic...");
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  /* Adjusting servos */
  ServoRoll.setPeriodHertz(50);
  ServoRoll.attach(PIN_SV_ROLL, 1000, 2000);
  ServoRoll.write(cVar::SV_CENTER);

  ServoPitch.setPeriodHertz(50);
  ServoPitch.attach(PIN_SV_PITCH, 1000, 2000);
  ServoPitch.write(cVar::SV_CENTER);

  
  #if IS_BLUETOOTH == 1
    BT_ESP.begin(ESP_NAME);

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
  #else 
    if (!LoRa.begin(LORA_FREQ)) {
      debugln("LoRa init failed");
      while (1) {
        ledError(500);
      }
    } else {
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(500E3); delay(10);
    }
  #endif

  delay(1000);
}


void calibration() {
  digitalWrite(PIN_LED, HIGH);
  #if IS_BLUETOOTH == 1
    BT_ESP.println("Calibrating ...");
  #else
    debugln("Calibrating ...");
  #endif
  
  MPU.verbose(true);
  delay(2000);

  MPU.calibrateAccelGyro();
  #if IS_BLUETOOTH == 1
    BT_ESP.println("Accelerometer calibrated");
  #else
    debugln("Accelerometer calibrated");
  #endif
  delay(2000);

  /* LED signal to start magnetometer manual calibration */
  ledError();
  ledError();
  ledError();
  digitalWrite(PIN_LED, HIGH);

  #if IS_BLUETOOTH == 1
    BT_ESP.println("Wave device in an figure eight");
  #else
    debugln("Wave device in an figure eight");
  #endif
  MPU.calibrateMag();
  #if IS_BLUETOOTH == 1
    BT_ESP.println("Magnetometer calibrated");
  #else
    debugln("Magnetometer calibrated");
  #endif
  delay(2000);

  MPU.verbose(false);
  digitalWrite(PIN_LED, LOW);
}


float controlPID(float _input, float _setpoint = 0.0, float _Kp = 1.0, float _Ki = 0.0, float _Kd = 0.0) {
  unsigned long timeActual = millis();
  
  float error = _setpoint - _input;
  float deltaTime = (timeActual - timeLast) / 1000.0;

  float proportional = _Kp * error;

  integral += error * deltaTime;
  integral = constrain(integral, PIDvar::INTEGRAL_WINDUP_MIN, PIDvar::INTEGRAL_WINDUP_MAX);   // ANTIWINDUP

  float derivative = (error - errorLast) / deltaTime;
  float derivativeFiltered = (PIDvar::DERIVATIVE_LPF_ALPHA * derivative) + ((1 - PIDvar::DERIVATIVE_LPF_ALPHA) * derivativeFiltered);   // LPF
  
  float _output = proportional + (_Ki * integral) + (_Kd * derivativeFiltered);

  errorLast = error;
  timeLast = timeActual;

  return _output;
}


void stabilizer(int _roll, int _pitch) {
  float rollPID = controlPID(-_roll, 0.0, PID_STABROLL_VAR[0], PID_STABROLL_VAR[1], PID_STABROLL_VAR[2]);
  float pitchPID = controlPID(-_pitch, 2.0, PID_STABPITCH_VAR[0], PID_STABPITCH_VAR[1], PID_STABPITCH_VAR[2]);

  ServoRoll.write(rollPID + cVar::SV_CENTER);
  ServoPitch.write(pitchPID + cVar::SV_CENTER);

  debug(rollPID); debug(",");
  debugln(pitchPID);
}


/* Turning control */
void controlTurning(int _roll, int _pitch, int _yaw_turn) {
  /* 
    _yaw_turn_bank: positive values means that the airplane must turn to the right,
    a negative value means left turn.
  */
  int16_t _yaw_turn_bank = (_yaw_turn < -180) ? (_yaw_turn + 360) : _yaw_turn;
  isTurn = (_yaw_turn_bank < 0) ? 1 : 2;      // 0 = Stabilizer, 1 = Right, 2 = Left

  if (isTurn == 1) {
    float rollPID = controlPID(-_roll, 20.0, PID_TURNROLL_VAR[0], PID_TURNROLL_VAR[1], PID_TURNROLL_VAR[2]);
    float pitchPID = controlPID(-_pitch, -15.0, PID_TURNPITCH_VAR[0], PID_TURNPITCH_VAR[1], PID_TURNPITCH_VAR[2]);
    ServoRoll.write(rollPID + cVar::SV_CENTER);
    ServoPitch.write(pitchPID + cVar::SV_CENTER);
  } else {
    float rollPID = controlPID(-_roll, -20.0, PID_TURNROLL_VAR[0], PID_TURNROLL_VAR[1], PID_TURNROLL_VAR[2]);
    float pitchPID = controlPID(-_pitch, -15.0, PID_TURNPITCH_VAR[0], PID_TURNPITCH_VAR[1], PID_TURNPITCH_VAR[2]);
    ServoRoll.write(rollPID + cVar::SV_CENTER);
    ServoPitch.write(pitchPID + cVar::SV_CENTER);
  }
}


/* Final approach pitch control */
// void controlApproach(int _roll, float _distanceGround) {
//   int8_t pitchControlled = ((float)(cVar::PITCH_MAX * kVar::APPROACH_PITCH) / cVar::ALTITUDE_THRESHOLD) * (cVar::ALTITUDE_THRESHOLD - _distanceGround);

//   ServoRoll.write((_roll * kVar::ROLL * -1) + cVar::SV_CENTER);
//   // ServoPitch.write(pitchControlled + cVar::SV_CENTER);
// } 

/* -------------------------------------------------- */

void loop() {
  if (MPU.update()) {
    int16_t roll, pitch, yaw;
    // float current, voltage;
    static uint32_t timePrevious = millis();

    if (Serial.available() > 0) {
      int PIDvalue = Serial.parseInt();
      if (Serial.read() == '\r') {}   // To stop blocking (just numbers)
      
      if (PIDvalue == 1) {
        Serial.println("Kp");
        while (1) {
          int value = Serial.parseFloat();
          PID_STABROLL_VAR[0] = value;
          break;
        }
      } else if (PIDvalue == 2) {
        while (1) {
          int value = Serial.parseFloat();
          PID_STABROLL_VAR[1] = value;
          break;
        }
      } else if (PIDvalue == 3) {
        while (1) {
          int value = Serial.parseFloat();
          PID_STABROLL_VAR[2] = value;
          break;
        }
      }
    }

    if (millis() > timePrevious + 25) {
      /* ----- ATTITUDE CONTROL ----- */
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
      
      roll = degrees(atan2(_roll_atan1, _roll_atan2));
      pitch = degrees(asin(2 * ((qw*qy) - (qx*qz))));
      yaw = degrees(atan2(_yaw_atan1, _yaw_atan2));
      yaw = (yaw < 0) ? (yaw + 360) : yaw;    // Yaw between 0 and 360

      int16_t yaw_turn = YAW_SETPOINT - yaw;


      /* ----- OTHER CALCULATIONS ----- */
      // float distanceGround = HCSR04_Measure();
      // current = analogRead(PIN_CURRENT);
      // voltage = analogRead(PIN_VOLTAGE);
      float altitude = BMP.readAltitude();
      float temperature = BMP.readTemperature();
      float ALTITUDE_HEIGHT = altitude - ALTITUDE_DEFINED;


      /* ----- MISSIONS ----- */
      /* 0 = Stabilizer, 1 = Turning, 2 = Approaching */
      isMission = (abs(yaw_turn) > YAW_TOLERANCE) ? 1 : 0;     // Turning detection
      // isMission = (distanceGround < cVar::ALTITUDE_THRESHOLD) ? 2 : isMission;    // Approach mission is prioritized over turning


      if (isArmed) {
        switch (isMission) {
          case 1:
            controlTurning(roll, pitch, yaw_turn);
            digitalWrite(PIN_LED, LOW);
            // analogWrite(PIN_BUZ, 0);
            break;

          // case 2:
          //   controlApproach(roll, distanceGround);
          //   digitalWrite(PIN_LED, LOW);
          //   analogWrite(PIN_BUZ, 0);
          //   break;

          default:
            stabilizer(roll, pitch);
            digitalWrite(PIN_LED, HIGH);
            // analogWrite(PIN_BUZ, 255);
            isTurn = 0;
            break;
        }
      }


      /* ----- DEBUGGING ----- */
      // debug(roll); debug(",");
      // debug(pitch); debug(",");
      // debug(yaw); debug(",");
      // debugln(isMission);

      String dataPacket = String(roll) + String(",") + String(pitch) + String(",") + String(yaw) + String(",") + 
                          String(altitude) + String(",") + String(ALTITUDE_HEIGHT) + String(",") + String(temperature) + String(",") + 
                          String(isTurn) + String(",") + String(isMission);


      /* ----- COMMUNICATION ----- */
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
      #else
        debugln(dataPacket);

        /* Sending data */
        LoRa.beginPacket();
        LoRa.print(dataPacket);
        LoRa.endPacket();
      #endif

      timePrevious = millis();
    }
  }
}