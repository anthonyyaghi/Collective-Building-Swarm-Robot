#include <Servo.h>
#include <QTRSensors.h>
#include <AutoPID.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

//PID settings and gains
#define OUTPUT_MIN -90
#define OUTPUT_MAX 90
#define KP .040
#define KI .0003
#define KD 0.002

//Sensors pins
#define rightPin 5
#define leftPin 4
#define rxPin 2
#define handSensor A0

//Arm settings
#define LOWER_ANGLE 109
#define UPPER_ANGLE 25

//States
#define STATE_FOLLOW_LINE 0
#define STATE_HOOK_BLOCK 1
#define STATE_DONE 2
#define STATE_FOLLOW_LINE_LOADED 3
#define STATE_UNLOAD 4
#define STATE_TEST 5

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {22, 24, 26, 28, 30, 32, 34, 36},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

double linePosition, setPoint, motorSpeed;
double leftPulse, leftDistance, rightPulse, rightDistance;

AutoPID myPID(&linePosition, &setPoint, &motorSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

Servo leftServo1;
Servo rightServo1;
Servo leftServo2;
Servo rightServo2;
Servo armServo;

double distArray[2];
int state;

String rxString;
String strAngle;

int ind1;
int ind2;
double angle, initAngle;

void setup() {
  pinMode(rightPin, INPUT);
  pinMode(leftPin, INPUT);
  pinMode(rxPin, OUTPUT);
  pinMode(handSensor, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  Serial.begin(9600);
  Serial.println("Done calibrating");

  Serial3.begin(9600);
  
  leftServo1.attach(8);
  rightServo1.attach(9);
  leftServo2.attach(10);
  rightServo2.attach(11);
  armServo.attach(12);
  armServo.write(UPPER_ANGLE);
  delay(700);
  
  setPoint = 3000;
  myPID.setTimeStep(0);
  setSpeedRight(0);
  setSpeedLeft(0);
  state = STATE_TEST;

  for(int i = 0; i < 10; i++){
    getAngle();
  }
}

void loop() {
  if(state == STATE_FOLLOW_LINE){
    getFrontalReads(distArray);
    if(distArray[0] < 25 && distArray[1] < 25){
      state = STATE_HOOK_BLOCK;
      setSpeedRight(0);
      setSpeedLeft(0);
      armServo.write(LOWER_ANGLE);
      delay(1000);
    }
    else{
      linePosition = qtrrc.readLine(sensorValues);
      myPID.run();
      setSpeedRight(45 + motorSpeed);
      setSpeedLeft(45 - motorSpeed);
    }
  }
  else if(state == STATE_HOOK_BLOCK){
    if(analogRead(handSensor) < 900){
      setSpeedRight(0);
      setSpeedLeft(0);
      armServo.write(UPPER_ANGLE);
      delay(1000);
      state = STATE_FOLLOW_LINE_LOADED;
    }
    else{
      setSpeedRight(90);
      setSpeedLeft(90);
    }
  }
  else if(state == STATE_DONE){
    setSpeedRight(0);
    setSpeedLeft(0);
  }
  else if(state == STATE_FOLLOW_LINE_LOADED){
    getFrontalReads(distArray);
    if(distArray[0] < 45 && distArray[1] < 45){
      state = STATE_UNLOAD;
      setSpeedRight(0);
      setSpeedLeft(0);
      delay(1000);
    }
    else{
      linePosition = qtrrc.readLine(sensorValues);
      myPID.run();
      setSpeedRight(45 + motorSpeed);
      setSpeedLeft(45 - motorSpeed);
    }
  }
  else if(state == STATE_UNLOAD){
    armServo.write(UPPER_ANGLE);
    for(int i = 0; i<20; i++){
      getFrontalReads(distArray);
    }
    while(distArray[0]-distArray[1] < -1 || distArray[0]-distArray[1] > 1){
      if(distArray[0]-distArray[1] < -1){
        setSpeedLeft(18);
        setSpeedRight(-18);
      }
      else if(distArray[0]-distArray[1] > 1){
        setSpeedLeft(-18);
        setSpeedRight(18);
      }
      getFrontalReads(distArray);
    }
    armServo.write(LOWER_ANGLE);
    delay(1000);
    setSpeedRight(90);
    setSpeedLeft(90);
    delay(700);
    setSpeedRight(0);
    setSpeedLeft(0);
    state = STATE_DONE;
  }
  else if(state == STATE_TEST){
    getAngle();
    if(strAngle != "na"){
      initAngle = strAngle.toDouble();
    }

    getAngle();
    if(strAngle != "na"){
      angle = strAngle.toDouble();
    }
    while(angle < (initAngle + 10)){
      setSpeedRight(90);
      setSpeedLeft(90);
      getAngle();
      if(strAngle != "na"){
        angle = strAngle.toDouble();
      }
    }

    while((angle > initAngle + 10)){
      setSpeedRight(30);
      setSpeedLeft(30);
      getAngle();
      if(strAngle != "na"){
        angle = strAngle.toDouble();
      }
    }

    setSpeedRight(0);
    setSpeedLeft(0);
    state = STATE_DONE;
  }
  
  /*Serial.print(distArray[0]);
  Serial.print("  ");
  Serial.println(distArray[1]);*/
}

void setSpeedLeft(int motorSpeed){
  leftServo1.write(90-motorSpeed);
  leftServo2.write(90-motorSpeed);
}

void setSpeedRight(int motorSpeed){
  rightServo1.write(90+motorSpeed);
  rightServo2.write(90+motorSpeed);
}

void getFrontalReads(double arr[]) {
  digitalWrite(rxPin, HIGH);
  //delayMicroseconds(25);
  rightPulse = pulseIn(rightPin, HIGH);
  rightDistance = rightPulse / 147 * 2.54;
  //digitalWrite(rxPin, LOW);

  leftPulse = pulseIn(leftPin, HIGH);
  leftDistance = leftPulse / 147 * 2.54;
  distArray[0] = rightDistance;
  distArray[1] = leftDistance;
}

void getAngle() {
  rxString=""; //clears variable for new input
  strAngle="";
  while(true){
    if (Serial3.available())  {
      char c = Serial3.read();  //gets one byte from serial buffer
      if (c == '\n') {
        ind1 = rxString.indexOf(',');  //finds location of first ,
        ind2 = rxString.indexOf(',', ind1+1 );   //finds location of second ,
        if(ind1 != -1 && ind2 != -2){
          strAngle = rxString.substring(ind1+1, ind2+1);   //captures second data String
        }
        else{
          strAngle = "na";
        }
        return;
      } 
      else {     
        rxString += c; //makes the string rxString
      }
    }
  }
}
