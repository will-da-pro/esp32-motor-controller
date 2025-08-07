#include <memory>

#define LEFT_MOTOR_PWM 7
#define LEFT_MOTOR_DIR 8
#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIR 5

#define LEFT_ENCODER_A 10
#define LEFT_ENCODER_B 9
#define RIGHT_ENCODER_A 37
#define RIGHT_ENCODER_B 38

#define TX_PIN 18 
#define RX_PIN 17

#define START_FLAG 0xA5
#define END_FLAG 0x5A

#define REPLY_REQUEST 0x20
#define DRIVE_REQUEST 0x30
#define STOP_REQUEST 0x31
#define ENCODER_REQUEST 0x40

#define ENCODER_RESPONSE 0x41


class PIDController {
public:
  PIDController();
  int calculatePID(int error);
  void reset();

  float kp = 1.0;
  float ki = 0.1;
  float kd = 0.01;

private:
  int prevError = 0;
  long prevTime;
  float errorIntegral = 0;
};


class Encoder {
public:
  Encoder(uint8_t pinA, uint8_t pinB);
  int getCount();
  int getSpeed();

  void interrupt();

private:
  uint8_t pinA;
  uint8_t pinB;

  int count = 0;

  time_t lastInterrupt = micros();
  int currentSpeed = 0;
};


class Motor {
public:
  Motor(uint8_t pwmPin, uint8_t dirPin, std::shared_ptr<Encoder> encoder);
  ~Motor();

  void drive(uint8_t speed, bool reversed);
  void drivePID(uint8_t speed, bool reversed);
  void stop();
  void update();
  bool isActive();

  PIDController pidController;

  int maxSpeed = 900;
  int minOutput = 75;

private:
  uint8_t pwmPin;
  uint8_t dirPin;

  uint targetSpeed = 0;
  bool reversed = false;

  bool active = false;

  std::shared_ptr<Encoder> encoder;
};


class Robot {
public:
  Robot();
  
  void drive(int8_t speed, int8_t turnRate);
  void straight(uint8_t speed, uint8_t distance);
  void turn(uint8_t speed, int distance);
  void stop();
  void update();

  std::shared_ptr<Encoder> leftEncoder;
  std::shared_ptr<Encoder> rightEncoder;

private:
  std::unique_ptr<Motor> leftMotor;
  std::unique_ptr<Motor> rightMotor;
};


/*
  PID Controller
*/


PIDController::PIDController() {
  this->prevTime = millis();
}

int PIDController::calculatePID(int error) {
  long currentTime = millis();

  float deltaT = float(currentTime - this->prevTime) / 1.0e6;
  this->prevTime = currentTime;

  float errorDerivative = float(error - this->prevError) / deltaT;
  this->prevError = error;

  this->errorIntegral += float(error) * deltaT;

  int result = error * this->kp + errorDerivative * this->kd + this->errorIntegral * this->ki;
  return result;
}

void PIDController::reset() {
  this->prevError = 0;
  this->prevTime = millis();
  this->errorIntegral = 0;
}


/*
  Encoder
*/


Encoder::Encoder(uint8_t pinA, uint8_t pinB) {
  this->pinA = pinA;
  this->pinB = pinB;

  pinMode(this->pinA, INPUT);
  pinMode(this->pinB, INPUT);
}

int Encoder::getCount() {
  return this->count;
}

int Encoder::getSpeed() {
  return this->currentSpeed;
}

void Encoder::interrupt() {
  uint8_t currentStateB = digitalRead(this->pinB);

  time_t ct = micros();
  int dt = micros() - this->lastInterrupt;
  this->lastInterrupt = ct;

  int speed = 1000000 / dt;

  if (currentStateB > 0) {
    count ++;
  }

  else {
    count --;
    speed = -speed;
  }

  this->currentSpeed = speed;
}


/*
  Motor
*/


Motor::Motor(uint8_t pwmPin, uint8_t dirPin, std::shared_ptr<Encoder> encoder) {
  this->pwmPin = pwmPin;
  this->dirPin = dirPin;

  this->encoder = encoder;

  pinMode(this->pwmPin, OUTPUT);
  pinMode(this->dirPin, OUTPUT);
}

Motor::~Motor() {
  this->stop();
}

void Motor::drive(uint8_t speed, bool reversed) {
  this->active = true;

  if (reversed) {
    speed = 255 - speed;
  }

  analogWrite(this->pwmPin, speed);
  digitalWrite(this->dirPin, reversed);
}

void Motor::drivePID(uint8_t speed, bool reversed) {
  this->active = true;

  this->targetSpeed = float(speed) / 255 * float(this->maxSpeed);
  this->reversed = reversed;

  this->pidController.reset();

  this->update();
}

void Motor::stop() {
  analogWrite(this->pwmPin, 0);
  digitalWrite(this->dirPin, LOW);

  this->active = false;
}

void Motor::update() {
  if (!this->active) {
    return;
  }

  int currentSpeed = this->encoder->getSpeed();
  int error = this->targetSpeed - currentSpeed;

  int pid = this->pidController.calculatePID(error);

  float rangeMult = float(this->minOutput) / 255;

  int output = rangeMult * pid + this->minOutput;

  if (output > 255) {
    output = 255;
  }

  else if (output < this->minOutput) {
    output = this->minOutput;
  }

  this->drive(uint8_t(output), this->reversed);
}

bool Motor::isActive() {
  return this->active;
}


/*
  Robot
*/


Robot::Robot() {
  this->leftEncoder = std::make_shared<Encoder>(LEFT_ENCODER_A, LEFT_ENCODER_B);
  this->rightEncoder = std::make_shared<Encoder>(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

  this->leftMotor = std::make_unique<Motor>(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, this->leftEncoder);
  this->rightMotor = std::make_unique<Motor>(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR, this->rightEncoder);
}

void Robot::drive(int8_t speed, int8_t turnRate) {
  uint8_t leftSpeed;
  bool leftReversed;

  uint8_t rightSpeed;
  bool rightReversed;

  if (speed + turnRate < 0) {
    leftSpeed = -speed - turnRate;
    leftReversed = true;
  }

  else {
    leftSpeed = speed + turnRate;
    leftReversed = false;
  }

  if (speed - turnRate < 0) {
    rightSpeed = -speed + turnRate;
    rightReversed = true;
  }

  else {
    rightSpeed = speed - turnRate;
    rightReversed = false;
  }

  this->leftMotor->drivePID(leftSpeed, leftReversed);
  this->rightMotor->drivePID(rightSpeed, rightReversed);
}

void Robot::straight(uint8_t speed, uint8_t distance) {
  this->stop();

  int startLeftCount = this->leftEncoder->getCount();
  int startRightCount = this->rightEncoder->getCount();

}

void Robot::stop() {
  this->leftMotor->stop();
  this->rightMotor->stop();
}

void Robot::update() {
  this->leftMotor->update();
  this->rightMotor->update();
}


Robot robot;

void leftEncoderInterrupt() {
  robot.leftEncoder->interrupt();
}

void rightEncoderInterrupt() {
  robot.rightEncoder->interrupt();
}

void sendInt(int val) {
  Serial1.print(char(val >> 24));
  Serial1.print(char(val >> 16 & 0xFF));
  Serial1.print(char(val >> 8 & 0xFF));
  Serial1.print(char(val & 0xFF));
}

void setup() {
  robot = Robot();

  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(36, OUTPUT);
  digitalWrite(36, HIGH);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderInterrupt, RISING);
}

void loop() {
  robot.update();

  if (Serial1.available()) {
    char* header = new char[2];
    Serial1.read(header, 2);

    if (header[0] != START_FLAG) {
      return;
    }

    char requestType = header[1];

    if (requestType == REPLY_REQUEST) {
      char messageSize = Serial1.read();

      String message = Serial1.readStringUntil(END_FLAG);
      if (message.length() != messageSize) {
        return;
      }

      Serial1.print(message);
    }

    else if (requestType == DRIVE_REQUEST) {
      /*
        DRIVE REQUEST STRUCTURE
        1 BYTE SPEED
        1 BYTE TURN RATE
      */

      char speed = Serial1.read();

      char turnRate = Serial1.read();

      robot.drive(speed, turnRate);

      Serial1.printf("%d %d OK\n", speed, turnRate);
    }

    else if (requestType == STOP_REQUEST) {
      robot.stop();
    }

    else if (requestType == ENCODER_REQUEST) {
      int leftEncoderValue = robot.leftEncoder->getCount();
      int rightEncoderValue = robot.rightEncoder->getCount();
      
      int leftEncoderSpeed = robot.leftEncoder->getSpeed();
      int rightEncoderSpeed = robot.rightEncoder->getSpeed();

      Serial1.print(char(START_FLAG));
      Serial1.print(char(ENCODER_RESPONSE));

      sendInt(leftEncoderValue);
      sendInt(rightEncoderValue);
      sendInt(leftEncoderSpeed);
      sendInt(rightEncoderSpeed);
    }
  }
}
