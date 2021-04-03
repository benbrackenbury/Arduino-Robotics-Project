#define DEBUG
#include <Servo.h>
#include <EEPROM.h>

//global variables
const int GREEN_LED = 7;
const int YELLOW_LED = 12;
const int RED_LED = 13;
const int SML = 6;
const int SMR = 5;
const int PBL = 4;
const int PBR = 2;
const int BOTH_PINS = 0;
const int RX = 2;
const int TX = 3;
const int WHEEL_DIAMETER = EEPROM[4];
int leftStopValue = EEPROM[0];
int rightStopValue = EEPROM[1];
int averageLdrValue = 0;
int leftOffset = EEPROM.read(2);
int rightOffset = EEPROM.read(3);
int averageLDRValuesWhite [3];
int averageLDRValuesBlack [3];
int leftLdrThreshold;
int middleLdrThreshold;
int rightLdrThreshold;
bool leftLdr, middleLdr, rightLdr;
bool ldrValues [3];
int junctions = 0;
bool checkIR = true;
unsigned long followLineStart;
bool cont = true;
//the two constant bools below help to make the code more readable
const bool REVERSE = true;
const bool FORWARDS = false;

Servo leftServo;
Servo rightServo;

//to help reading from EEPROM
unsigned int readUIValue(int eepromAddress) {
  unsigned int uiVal;
  EEPROM.get(eepromAddress, uiVal);

  return uiVal;
}

void setLEDs(int green_state, int yellow_state, int red_state) {
 digitalWrite(GREEN_LED, green_state);
 digitalWrite(YELLOW_LED, yellow_state);
 digitalWrite(RED_LED, red_state);
}


void flashLED(int led) {
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
}

void flashRedLED() { flashLED(RED_LED); }
void flashYellowLED() { flashLED(YELLOW_LED); }
void flashGreenLED() { flashLED(GREEN_LED); }

/*
*waits for a key to be pressed, then
 either calls another function or returns if
 _ is provided as a parameter
*/
void waitKey(int pin, void (*next)()) {
  if (pin == BOTH_PINS) {
    for (int i=0; i<21; i++) {
      if(digitalRead(PBL) == LOW && digitalRead(PBR) == LOW) {
        (*next)();
      }
      delay(1000);
    }
  }
  
  while (digitalRead(pin) == HIGH) {} //wait for press
  delay(20);
  while (digitalRead(pin) == LOW) {}  //wait for release
  delay(20);
  (*next)();
}

/*
 * does nothing and returns to parent
 * function. Used for waitKey.
*/
void _() {
  return;
}
void turnAngle(int degrees) {
  setSpeed(0, 0);
  if (degrees < 0) {
    setSpeed(-20, 20);
  } else {
    setSpeed(20, -20);
  }
  int turnDuration = abs(22 * degrees);
  delay(turnDuration);
  setSpeed(0, 0);
}

void starightLine() {
  //forwards
  setSpeed(20, 20);
  delay(10000);
  setSpeed(0, 0);
  delay(500);

  //turn 180°
  setSpeed(20, -20);
  delay(3400);
  setSpeed(0, 0);

  //drive back
  delay(500);
  setSpeed(20, 20);
  delay(10000);
  setSpeed(0, 0);
  
  setLEDs(0,1,1);
}

void turns() {
  //90 degress cw
  turnAngle(90);
  delay(1000);
  //90 degress ccw
  turnAngle(-90);
  delay(1000);
}

void letsDance() {
  setLEDs(0,0,0);
  for (int i; i<7; i++) {
    setLEDs(1,0,1);
    //drive forwards for 5s
    setSpeed(0, 0);
    setSpeed(20, 20);
    delay(2500);
    setSpeed(0,0);
  
    //90 degress cw
    setLEDs(0,1,0);
    turnAngle(60);
    setLEDs(0,0,0);
  }
}

void drive(bool reverse=FORWARDS, bool timed=true, bool count=false) {
  while (cont) {
    //check PBR - used for initated stage 4
    bool firstCheck = false;
    bool secondCheck = false;
    if(digitalRead(PBL) == LOW && digitalRead(PBR) == LOW) { firstCheck == true; }
    delay(200);
    if(digitalRead(PBL) == LOW && digitalRead(PBR) == LOW) { secondCheck == true; }
    if(firstCheck && secondCheck) {
      cont = false;
      setSpeed(0, 0);
      setLEDs(0,0,0);
      break;
    }

    //stops loop after 12 seconds. ~50cm
    if (timed && millis() - followLineStart  > 12000) {
      cont = false;
    }
    //stops loop if 3 junctions passed - used in stage 4
    if(count && junctions == 3) {
      //passed 3 junctions
      delay(2500); //give robot enough space to turn round
      cont = false;
      setSpeed(0, 0);
      break;
    }
    //send ir signal
    tone(TX, 38000);
    if (checkIR) { //if we want IR to be checked - we don't while its turning
      delay(300);
      if (digitalRead(RX) == LOW){
        //stop and turn around
        setSpeed(0, 0);
        setLEDs(0,0,1);
        delay(1000);
        setLEDs(1,1,1);
        delay(1000);
        turn180();
        checkIR = false;
        checkIR = true;
      } else {
        //set direction
        if(reverse) {
          checkLDRsReverse();
        } else {
          checkLDRs();
        }
      }
    } else {
      //set direction
      if(reverse) {
        checkLDRsReverse();
      } else {
        checkLDRs();
      }
    }
    
    noTone(TX); //stop IR
    delay(300);
  }
}

void followPath() {
  followLineStart = millis(); //start timer
  drive();

  //gone on for ~10 seconds, so turn 180
  setSpeed(0,0);
  delay(500);
  setLEDs(0,1,0);
  turn180();
  followLineStart = millis();

  //drive again
  cont = true;
  followLineStart = 0;
  delay(100);
  followLineStart = millis();
  setLEDs(0,1,0);
  drive();

  //reverse
  delay(1000);
  setLEDs(1,0,1);
  setSpeed(0,0);
  delay(1000);
  drive(REVERSE);
}

void countJunctions() {
  setLEDs(1,0,1);
  junctions = 0;
  drive(FORWARDS, false, true); //drive forwards, without timing, and count the junctions
  junctions = 0;
  turnAngle(160);
  delay(1000);
  drive(FORWARDS, false, true);
}

void turn180() {
  setSpeed(0, 0);
  setSpeed(20, -20);
  delay(3400);
  setSpeed(0, 0);
}

void checkLDRs() {
  //read values
  if(analogRead(A2) < rightLdrThreshold) {
    leftLdr = false;
    ldrValues[0] = leftLdr;
  } else {
    leftLdr = true;
    ldrValues[0] = leftLdr;
  }

  if(analogRead(A1) < middleLdrThreshold) {
    middleLdr = false;
    ldrValues[1] = middleLdr;
  } else {
    middleLdr = true;
    ldrValues[1] = middleLdr;
  }

  if(analogRead(A0) < leftLdrThreshold) {
    rightLdr = false;
    ldrValues[2] = rightLdr;
  } else {
    rightLdr = true;
    ldrValues[2] = rightLdr;
  }

//compare read values to threshold
  if (ldrValues[0] == 1 && ldrValues[1] == 0 && ldrValues[2] == 1) {
    //on track
    setSpeed(20, 20);
    setLEDs(1, 0, 0);
  }
  if (ldrValues[0] == 0 && ldrValues[1] == 1 && ldrValues[2] == 1) {
    //turn left
    setSpeed(-20, 20);
    delay(300);
    setSpeed(20, 20);
    setLEDs(1, 0, 0);
  }
  if (ldrValues[0] == 0 && ldrValues[1] == 0 && ldrValues[2] == 1) {
    //turn left
    setSpeed(-20, 20);
    delay(300);
    setSpeed(20, 20);
    setLEDs(1, 0, 0);
  }
  if (ldrValues[0] == 1 && ldrValues[2] == 0) {
    //turn right
    setSpeed(20, -20);
    delay(300);
    setSpeed(20, 20);
    setLEDs(1, 0, 0);
  }

  if (ldrValues[0] == 0 && ldrValues[1] == 0 && ldrValues[2] == 0) {
    junctions++;
    setSpeed(20, 20);
    setLEDs(1, 1, 1);  
  }
}

//same as checkLDRs, but values are inverted for reverse driving
void checkLDRsReverse() {
  setLEDs(1,0,1);
  if(analogRead(A2) < rightLdrThreshold) {
    leftLdr = false;
    ldrValues[0] = leftLdr;
  } else {
    leftLdr = true;
    ldrValues[0] = leftLdr;
  }

  if(analogRead(A1) < middleLdrThreshold) {
    middleLdr = false;
    ldrValues[1] = middleLdr;
  } else {
    middleLdr = true;
    ldrValues[1] = middleLdr;
  }

  if(analogRead(A0) < leftLdrThreshold) {
    rightLdr = false;
    ldrValues[2] = rightLdr;
  } else {
    rightLdr = true;
    ldrValues[2] = rightLdr;
  }

  if (ldrValues[0] == 1 && ldrValues[1] == 0 && ldrValues[2] == 1) {
    //on track
    setSpeed(-20, -20);
    setLEDs(1, 0, 0);
  }
  if (ldrValues[0] == 0 && ldrValues[1] == 1 && ldrValues[2] == 1) {
    //turn left
    setSpeed(20, -20);
    delay(500);
    setSpeed(-20, -20);
    setLEDs(1, 0, 0);
  }
  if (ldrValues[0] == 0 && ldrValues[1] == 0 && ldrValues[2] == 1) {
    //turn left
    setSpeed(20, -20);
    delay(500);
    setSpeed(-20, -20);
    setLEDs(1, 0, 0);
  }
  if (ldrValues[0] == 1 && ldrValues[2] == 0) {
    //turn right
    setSpeed(-20, 20);
    delay(500);
    setSpeed(-20, -20);
    setLEDs(1, 0, 0);
  }

  if (ldrValues[0] == 0 && ldrValues[1] == 0 && ldrValues[2] == 0) {
    junctions++;
    setSpeed(-20, -20);
    setLEDs(1, 1, 1);  
  }
}


void calibrateLeftServo() {
  while(digitalRead(PBL) == HIGH) { //wait for left button press
    leftServo.write(leftStopValue);
    delay(3000);
    leftStopValue--;
  }
  leftStopValue ++;
  return;
}

void calibrateRightServo() {
  while(digitalRead(PBR) == HIGH) { //wait for right button press
    rightServo.write(rightStopValue);
    delay(3000);
    rightStopValue++;
  }
  rightStopValue --;
  return;
}

void calibrateServos() {
  setLEDs(0,0,1);
  delay(1000);
  calibrateLeftServo();
  setLEDs(0,1,0);
  calibrateRightServo();
  EEPROM.update(0, leftStopValue);
  EEPROM.update(1, rightStopValue);
  setLEDs(1,0,0);
}

void setSpeed(float left, float right){
  if (left > 0) {
    left += leftOffset;
  } else if ( left < 0) {
    left -= leftOffset;
  }

  if (right > 0) {
    right += rightOffset;
  } else if ( left < 0) {
    right -= rightOffset;
  }
  
  leftServo.write(leftStopValue + left);
  rightServo.write(rightStopValue - right);
}

void calibrateLDRsOnWhite() {
  setLEDs(1,1,1);
  
  //left: A0
  for (int i=0; i<51; i++) { averageLDRValuesWhite[0] += analogRead(A0); }
  averageLDRValuesWhite[0] /= 50;
  EEPROM.put(10, averageLDRValuesWhite[0]);
  
  //middle: A1
  for (int i=0; i<51; i++) { averageLDRValuesWhite[1] += analogRead(A1); }
  averageLDRValuesWhite[1] /= 50;
  EEPROM.put(13, averageLDRValuesWhite[1]);

  //right: A2
  for (int i=0; i<51; i++) { averageLDRValuesWhite[2] += analogRead(A2); }
  averageLDRValuesWhite[2] /= 50;
  EEPROM.put(16, averageLDRValuesWhite);
  delay(1000);
  setLEDs(1,0,0);
}

void calibrateLDRsOnBlack() {
  setLEDs(1,1,1);
  
  //left: A0
  for (int i=0; i<51; i++) { averageLDRValuesBlack[0] += analogRead(A0); }
  averageLDRValuesBlack[0] /= 50;
  EEPROM.put(10, averageLDRValuesWhite[0]);
  
  //middle: A1
  for (int i=0; i<51; i++) { averageLDRValuesBlack[1] += analogRead(A1); }
  averageLDRValuesBlack[1] /= 50;
  EEPROM.put(13, averageLDRValuesWhite[1]);

  //right: A2
  for (int i=0; i<51; i++) { averageLDRValuesBlack[2] += analogRead(A2); }
  averageLDRValuesBlack[2] /= 50;
  EEPROM.put(16, averageLDRValuesBlack);
  delay(1000);
  setLEDs(1,0,0);
}

void calculateThresholds() {
  int leftWhite = averageLDRValuesWhite[0];
  int leftBlack = averageLDRValuesBlack[0];
  int middleWhite = averageLDRValuesWhite[1];
  int middleBlack = averageLDRValuesBlack[2];
  int rightWhite = averageLDRValuesWhite[2];
  int rightBlack = averageLDRValuesBlack[2];

  leftLdrThreshold = (leftWhite + leftBlack) / 2;
  middleLdrThreshold = (middleWhite + middleBlack) / 2;
  rightLdrThreshold = (rightWhite + rightBlack) / 2;
}

void setup() {
  Serial.begin(9600);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(PBL, INPUT);
  pinMode(PBR, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT); 
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  leftServo.attach(SML);
  rightServo.attach(SMR);

  setSpeed(0,0);
  setLEDs(0,0,1);

//allows the user 5 seconds to initate the servo stopping value callibration
  for (int i=0; i<6; i++) {
    if(digitalRead(PBL) == LOW && digitalRead(PBR) == LOW) {
      calibrateServos();
    }
    delay(1000);
  }

  setLEDs(0,1,1);

    waitKey(PBL, starightLine);
    waitKey(PBL, turns);
    waitKey(PBL, letsDance);
    waitKey(PBL, calibrateLDRsOnWhite);
    waitKey(PBL, calibrateLDRsOnBlack);
    calculateThresholds();
    waitKey(PBR, followPath);
    waitKey(PBL, followPath);
    waitKey(PBL, objectDetection);
    countJunctions();
}

void loop() {
  
}
