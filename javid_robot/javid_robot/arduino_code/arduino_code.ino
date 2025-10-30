const int leftLed = 3;
const int rightLed = 11;
const int LM1 = 9;
const int LM2 = 10;
const int RM1 = 5;
const int RM2 = 6;

void setup() {
  Serial.begin(9600); // Set the baud rate to match with ROS2
  pinMode(leftLed, OUTPUT); // Set the LED pin as an output
  pinMode(rightLed, OUTPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    processCommand(receivedChar);
  }
}

void processCommand(char command) {
  switch (command) {
    case '1':
      setLeds(true, false);
      Serial.println("Left Led ON");
      break;

    case '0':
      setLeds(false, true);
      Serial.println("Right Led ON");
      break;

    case 'q':
      setMotors(HIGH, LOW, LOW, LOW);
      Serial.println("LM1 ON");
      break;

    case 'w':
      setMotors(LOW, HIGH, LOW, LOW);
      Serial.println("LM2 ON");
      break;

    case 'a':
      setMotors(LOW, LOW, HIGH, LOW);
      Serial.println("RM1 ON");
      break;

    case 's':
      setMotors(LOW, LOW, LOW, HIGH);
      Serial.println("RM2 ON");
      break;

    case 't':
      setLeds(false, false);
      setMotors(LOW, LOW, LOW, LOW);
      Serial.println("MOTORS OFF, LEDs OFF");
      break;
  }
}

void setLeds(bool left, bool right) {
  digitalWrite(leftLed, left ? HIGH : LOW);
  digitalWrite(rightLed, right ? HIGH : LOW);
}

void setMotors(int lm1, int lm2, int rm1, int rm2) {
  digitalWrite(LM1, lm1);
  digitalWrite(LM2, lm2);
  digitalWrite(RM1, rm1);
  digitalWrite(RM2, rm2);
}
