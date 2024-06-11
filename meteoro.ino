// https://arduinogetstarted.com/tutorials/arduino-light-sensor

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

const int ultrasonicTriggerPin = 11;
const int ultrasonicEchoPin = 12;

int stateDist = LOW;
int stateLight = LOW;
unsigned long prevDistTime = 0;
unsigned long prevLightTime = 0;

int ON_LIGHT_MIL = 120;
int OFF_LIGHT_MIL = 140;
int ON_DISTANCE_MIL = 100;
int OFF_DISTANCE_MIL = 120;

bool activateChange(unsigned long currMillis, unsigned long prevMillis, int millis) {
  return currMillis - prevMillis >= millis;
}

class LightSensor{
  
  int rightSensor;
  int leftSensor;
  int onLightMil;
  int offLightMil;
  int stateLight;
  unsigned long prevLightTime;
  unsigned long leftSensedValue;
  unsigned long rightSensedValue;

  public: LightSensor(int rSensor, int lSensor, unsigned long onMil, unsigned long offMil) {
    rightSensor = rSensor; // A0
    leftSensor = lSensor; // A1
    onLightMil = onMil;
    offLightMil = offMil;

    stateLight = LOW;
    prevLightTime = 0;
    rightSensedValue = 0;
    leftSensedValue = 0;
  }

  void Update(){

    unsigned long currMil = millis();

    if (stateLight == LOW && activateChange(currMil, prevLightTime, onLightMil)) {
      stateLight = HIGH;
      prevLightTime = currMil;
      leftSensedValue = analogRead(leftSensor);
      rightSensedValue = analogRead(rightSensor);
    } else if (stateLight == HIGH && activateChange(currMil, prevLightTime, offLightMil)) {
      stateLight = LOW;
      prevLightTime = currMil;
    }
  }

  unsigned long getLeftSensedValue(){
    return leftSensedValue;
  }

  unsigned long getRightSensedValue(){
    return rightSensedValue;
  }

};

class DistSensor{

  int onDisttMil;
  int offDitMil;
  int stateDist;
  unsigned long prevDistTime;
  int cm;

  public: DistSensor(unsigned long onMil, unsigned long offMil) {
    onDisttMil = onMil;
    offDitMil = offMil;

    stateDist = LOW;
    prevDistTime = 0;
  }

  void Update(){

    unsigned long currMil = millis();

    if (stateDist == LOW && activateChange(currMil, prevDistTime, onDisttMil)) {
      stateDist = HIGH;
      prevDistTime = currMil;
      cm = measureDistance();
    } else if (stateDist == HIGH && activateChange(currMil, prevDistTime, offDitMil)) {
      stateDist = LOW;
      prevDistTime = currMil;
    }
  }

  int measureDistance() {
    digitalWrite(ultrasonicTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicTriggerPin, LOW);
    long duration = pulseIn(ultrasonicEchoPin, HIGH);
    return microsecondsToCentimeters(duration);
  }

  long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2;
  }

  int getCm(){
    return cm;
  }

};

DistSensor dSensor(ON_DISTANCE_MIL, OFF_DISTANCE_MIL);
LightSensor lSensor(A0, A1, ON_LIGHT_MIL, OFF_LIGHT_MIL);

void setup() {

  Serial.begin(9600);
  pinMode(ultrasonicTriggerPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {

  dSensor.Update();
  lSensor.Update();

  Serial.print("Distance: ");
  Serial.println(dSensor.getCm());
  if (dSensor.getCm() <= 30) {
    stopMotor();
    turnAround();
  } else {
    Serial.print("Sensed values: ");
    Serial.print(lSensor.getLeftSensedValue());
    Serial.print(" and ");
    Serial.println(lSensor.getRightSensedValue());
    moveForward(lSensor.getLeftSensedValue(), lSensor.getRightSensedValue());
  }

}

void moveForward(int left, int right) {
  
  // Turn on motor A & B
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // Set motors speed
  // For PWM maximum possible values are 0 to 255
  analogWrite(enA, 128);
  analogWrite(enB, 128);


  int threshold = abs(right - left);

  Serial.print("Threshold: ");
  Serial.println(threshold);

  if (threshold > 150) {
    if (left < right) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);  // Set motors speed
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 170);
      analogWrite(enB, 0);

    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      // Set motors speed
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 0);
      analogWrite(enB, 170);

    }
  }
}

void stopMotor() {
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void moveBackwards() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnAround() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  accelerate(170);
}

void accelerate(int limitSpeed) {
  // Accelerate from zero to maximum speed
  for (int i = 0; i < limitSpeed; i++) {
    analogWrite(enA, i);
    analogWrite(enB, i);
  }
}

void decelerate(int initialSpeed) {
  // Decelerate from maximum speed to zero
  for (int i = initialSpeed; i >= 0; --i) {
    analogWrite(enA, i);
    analogWrite(enB, i);
  }
}
