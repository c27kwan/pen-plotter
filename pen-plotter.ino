#include <TMCStepper.h>

#include <Wire.h>
#include "ADS1X15.h"
#include <Adafruit_PWMServoDriver.h>


constexpr pin_size_t pin_chipSelA = 21; // Mot0
constexpr pin_size_t pin_chipSelB = 18; // Enc0
constexpr pin_size_t pin_chipSelC = 25; // Mot1
constexpr pin_size_t pin_chipSelD = 24; // Enc1

constexpr pin_size_t pin_SPI0SCK = 22;
constexpr pin_size_t pin_SPI0MOSI = 19;
constexpr pin_size_t pin_SPI0MISO = 20;

constexpr pin_size_t in1Pin = 1;
constexpr pin_size_t in2Pin = 0;

// Stepper Motor OConfig
constexpr uint16_t MOTOR_CURRENT = 500;
constexpr uint8_t DEFAULT_IRUN = 8;
constexpr uint8_t DEFAULT_IHOLD = 8;
constexpr uint8_t TMC_TOFFRUN = 4;

#define SPEED_RPS 1.0
#define RATIO_ACCEL 5.0
#define RAIIO_DECEL 5.0

constexpr float FULL_STEPS_PER_REV = 200.0;
constexpr float MICROSTEP_FACTOR = 256.0;

constexpr float DEFAULT_VEL = (FULL_STEPS_PER_REV * MICROSTEP_FACTOR * SPEED_RPS);
constexpr float DEFAULT_ACCEL = (FULL_STEPS_PER_REV * MICROSTEP_FACTOR / RATIO_ACCEL);
constexpr float DEFAULT_DECEL = (FULL_STEPS_PER_REV * MICROSTEP_FACTOR / RAIIO_DECEL);

constexpr float RS = 0.05;

TMC5160Stepper stepperDriver(pin_chipSelA, RS);

Adafruit_PWMServoDriver linearDriver(PCA9685_I2C_ADDRESS, Wire1);
ADS1015 ADS(0x48, &Wire1);


double curr_radian = M_PI / 2.0;

void setupMotor() {
  stepperDriver.setSPISpeed(1000000);
  stepperDriver.reset();
  stepperDriver.toff(0);
  stepperDriver.rms_current(MOTOR_CURRENT);
  stepperDriver.ihold(DEFAULT_IHOLD);
  stepperDriver.irun(DEFAULT_IRUN);
  stepperDriver.en_pwm_mode(false);
  stepperDriver.VSTOP(10);
  stepperDriver.RAMPMODE(0);
  stepperDriver.TZEROWAIT(0);

  stepperDriver.shaft(0);
  stepperDriver.en_softstop(0);
  stepperDriver.shortdelay(true);
  stepperDriver.shortfilter(2);
  //
  // Sets the internal motion profile —- see datasheet
  stepperDriver.v1(0); // Use Trapezoid Only, disable first ramps
  stepperDriver.a1(DEFAULT_ACCEL);
  stepperDriver.d1(DEFAULT_DECEL);
  stepperDriver.AMAX(DEFAULT_ACCEL);
  stepperDriver.VMAX(DEFAULT_VEL);
  stepperDriver.DMAX(DEFAULT_DECEL);

  stepperDriver.toff(TMC_TOFFRUN);
}

void setupI2C() {
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();

  Serial.println("Wire 1 Begin");
  if (!ADS.begin())
    Serial.println("ADS Error");

  if (!linearDriver.begin())
    Serial.println("PWM Error");

  linearDriver.setPin(in1Pin, 0);
  linearDriver.setPin(in2Pin, 0);

  Serial.println("I2C Done");
}

void setup() {
  pinMode(pin_chipSelA, OUTPUT);
  SPI.setTX(pin_SPI0MOSI);
  SPI.setRX(pin_SPI0MISO);
  SPI.setSCK(pin_SPI0SCK);

  SPI.begin();

  Serial.begin(9600);
  while (!Serial); // Wait for USB monitor to open
  Serial.println("Online");

  setupMotor();
  setupI2C();

  if (stepperDriver.test_connection() != 0)
  {
    Serial.println("Driver not connected");
    while (1);
  }
}


int readPosition() {
  return ADS.readADC(1);
}

void linearRetract(uint16_t speed = 4095)
{
  linearDriver.setPin(in1Pin, speed);
  linearDriver.setPin(in2Pin, 0);
}

void linearExtend(uint16_t speed = 4095)
{
  linearDriver.setPin(in1Pin, 0);
  linearDriver.setPin(in2Pin, speed);
}

void linearStop() {
  linearDriver.setPin(in1Pin, 0);
  linearDriver.setPin(in2Pin, 0);
}

// Position [0, 1004]
void moveActuator(int16_t position) {
  if (position < 0 || position > 1004) {
    Serial.println("yikes out of bound");
    return;
  }
  uint16_t speed = 4095; // This needs to be low enough that we can represent unit = 1.

  Serial.println("Position");
  Serial.println(readPosition());
  int initialPosition = readPosition();

  // The current APIs extend and retract just sets the movement. Like an object that stays in motion.
  // The linearStop is what stops its movement.
  if (position > initialPosition) {
    linearExtend(speed);
  } else {
    linearRetract(speed);
  }
  // busy wait loop
  while(readPosition() != position) {}
  linearStop();
}

void moveAndRotateTogether(int16_t position, int32_t microsteps) {
  if (position < 0 || position > 1004) {
    Serial.println("yikes out of bound");
    return;
  }
  uint16_t speed = 4095; // This needs to be low enough that we can represent unit = 1.

  Serial.println("Position");
  Serial.println(readPosition());
  int initialPosition = readPosition();

  // The current APIs extend and retract just sets the movement. Like an object that stays in motion.
  // The linearStop is what stops its movement.
  if (position > initialPosition) {
    linearExtend(speed);
  } else {
    linearRetract(speed);
  }
  stepperDriver.XTARGET(stepperDriver.XACTUAL() + microsteps);
  while (readPosition() != position) {}
  linearStop();
  while (!stepperDriver.position_reached()) {}
}

void moveAndRotate(int16_t position, int32_t microsteps) {
  stepperDriver.XTARGET(stepperDriver.XACTUAL() + microsteps);
  moveActuator(position);
  while (!stepperDriver.position_reached()) {}
  Serial.print("Final rotation position: ");
  Serial.println(stepperDriver.XACTUAL());
}

int degreesToMicrosteps(int degrees) {
  double radian = degrees * M_PI / 180;
  return radianToMicrosteps(radian);
}

int radianToMicrosteps(double radian) {
  int MAX_STEPS = 200 * 256 * 20;
  return (radian * MAX_STEPS) / (2 * M_PI);
}

// x in [-900, 900], y in [100, 1000]
void moveToXY(int x, int y) {
  int ARM_LENGTH = 516;
  int X_MIN = -400;
  int X_MAX = 400;
  int Y_MIN = ARM_LENGTH;
  int Y_MAX = ARM_LENGTH + 1000;
  if (y < ARM_LENGTH || x > X_MAX || x < X_MIN || y > Y_MAX) {
    return;
  }
  double angle = atan2(y, x);

  int length = (int) sqrt(x * x + y * y);
  Serial.print("Length: ");
  Serial.println(length);

  double radian = curr_radian - angle;
  curr_radian = angle;
  Serial.print("Radian: ");
  Serial.println(curr_radian);
  Serial.println(radian);

  int microsteps = radianToMicrosteps(radian);
  Serial.print("Microsteps: ");
  Serial.println(microsteps);

  // moveAndRotate(length - ARM_LENGTH, microsteps);
  moveAndRotateTogether(length - ARM_LENGTH, microsteps);
}

void rotateSteps(int32_t microsteps) {
  rotateToTarget(stepperDriver.XACTUAL() + microsteps);
}

void rotateToTarget(int32_t targetPositionInMicrosteps) {
  stepperDriver.XTARGET(targetPositionInMicrosteps);
  while (!stepperDriver.position_reached()) {
    // Serial.println("Moving");
    // delay(1000);
  }
  Serial.print("Final rotation position: ");
  Serial.println(stepperDriver.XACTUAL());
}

// Example of using the functions to control things
void loop() {
  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');  // Read until newline

    Serial.print("You entered: ");
    Serial.println(input);
    String parsedInput[5];
    uint16_t currIndex = 0;
    size_t tokenIndex = 0;
    while (currIndex < input.length()) {
      int endIndex = input.indexOf(" ", currIndex);
      if (endIndex < 0) {
        endIndex = input.length();
      }
      String token = input.substring(currIndex, endIndex);
      parsedInput[tokenIndex++] = token;
      // Serial.println("Token: ");
      // Serial.println(token);
      currIndex = endIndex + 1;
    }

    switch(parsedInput[0].charAt(0)) {
      case 'd':
      case 'D': {
        int degrees = parsedInput[1].toInt();
        int microsteps = degreesToMicrosteps(degrees);
        rotateSteps(microsteps);
        break;
      }
      case 'm':
      case 'M': {
        uint16_t position = parsedInput[1].toInt();
        moveActuator(position);
       break;
      }
      case 'r':
      case 'R': {
        int microsteps = parsedInput[1].toInt();
        rotateSteps(microsteps);
        break;
      }
      case 't':
      case 'T': {
        int microsteps = parsedInput[1].toInt();
        rotateToTarget(microsteps);
        break;
      }
      case 'z':
      case 'Z': {
        int x = parsedInput[1].toInt();
        int y = parsedInput[2].toInt();
        moveToXY(x, y);
        break;
      }
    }
  }
}
