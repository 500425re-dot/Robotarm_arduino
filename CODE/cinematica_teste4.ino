//3d kinematics

#include <Servo.h>
#include <LiquidCrystal_I2C.h> 
#include <Wire.h> 

//kinematics definitions
#define L1 14
#define L2 12
#define L3 9
float x;
float y;
float z;
float psi;
bool debug;
int delay2 = 2500;
int shortDelay = 100;

//ldc
#define adress  0x27 // Endereços comuns: 0x27, 0x3F
#define collums   16
#define rows    2

//inputs
#define but 3
#define sw 4
#define vrx A0
#define vry A1

//servos
#define servo1 5
#define servo2 6
#define servo3 9
#define servo4 10
#define servo5 11
#define baseHomePosition 0
#define shoulderHomePosition 140
#define elbowHomePosition 180
#define wristHomePosition 60
#define gripperHomePosition 90

//inputs
unsigned long debounce;
int vrxValue;
int vryValue;
bool butState;
bool swState;

//two options
int elbowValueA;
int shoulderValueA;
int elbowValueB;
int shoulderValueB;

//others
int valueReceived;
unsigned long pause;
bool turn;
int j;
int k;
int i;


//objects
LiquidCrystal_I2C lcd(adress, collums, rows);
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;   //inverted
Servo gripper;

//functions
void IK(float x, float y, float psi, bool debug);
// "test": int percurso(int i, int pause, int max, int increment = 1);

void setup() {
  
  lcd.init(); 
  lcd.backlight(); 
  lcd.clear();

  pinMode(but, INPUT_PULLUP);
  pinMode(sw, INPUT_PULLUP);
  pinMode(vrx, INPUT);
  pinMode(vry, INPUT);

  //servo configurations and home position
  base.attach(servo1);
  shoulder.attach(servo4);
  elbow.attach(servo2);
  wrist.attach(servo3);
  gripper.attach(servo5);
  base.write(baseHomePosition);
  shoulder.write(shoulderHomePosition);
  elbow.write(elbowHomePosition);
  wrist.write(wristHomePosition);
  gripper.write(gripperHomePosition);

  //serial communication
  Serial.begin(9600); 
}

void loop() {

  lcd.setCursor(0, 0);
  lcd.print("shoulder: elbow:");
  lcd.setCursor(0, 1); 
  lcd.print(404);
  lcd.setCursor(8, 1); 
  lcd.print(404);

  //take values in the serial monitor
  if(false){ 
      switch(valueReceived){ 
      case 0:
          Serial.println("x: ");
          if(Serial.available() > 0){
              x = Serial.parseFloat();
            valueReceived = 1;
          }
          break;

      case 1:    
          Serial.println("y: ");
          if(Serial.available() > 0){
              y = Serial.parseFloat();
            valueReceived = 2;
          }
          break;

      case 2:
        Serial.println("gripper angles: ");
        if(Serial.available() > 0){
            psi = Serial.parseInt();
          valueReceived = 0;
        }
        break;
      }
  }

  //kinematics
  
    gripper.write(130);
    IK(29, -5, 0, -90, true);
    delay(delay2 );
    IK(20,20, 0, -90, true);
    delay(delay2);
    for(i = 0; i<= 10; i++){
      IK(20, 20, i, 0, true);
      delay(shortDelay);
    }
    for(i = 20; i>= -5; i--){
      IK(20, i, 10, -90, true);
      delay(shortDelay);
    }
    IK(17,-5, 10, -90, true);
    delay(shortDelay);
    for(i = -5; i<= 20; i++){
      IK(20, i, 10, -90, true);
      delay(shortDelay);
    }
    IK(20,20, 10, -90, true);
    delay(delay2);
    for(i = 20; i>= -5; i--){
      IK(20, i, 10, -90, true);
      delay(shortDelay);
    }
    for(i = 10; i<= -10; i++){
      IK(20, -5, i, -90, true);
      delay(shortDelay);
    }
    IK(20,-5, -10, -90, true);
    delay(delay2);
    for(i = -5; i<= 20; i++){
      IK(20, i, -10, -90, true);
      delay(shortDelay);
    }
    for(i = -10; i<= 0; i++){
      IK(20, 20, i, -90, true);
      delay(shortDelay);
    }
    delay(delay2);
  
}

//inverse kinematics function
void IK(float x, float y, float z, float psi, bool debug){
  double theta4 = atan2(z,x) ;
  double x2 = x/(cos(theta4));
  double x3 = x2 - 9.0*cos(psi*DEG_TO_RAD);
  double y3 = y - 9.0*sin(psi*DEG_TO_RAD);
  double beta = atan2(y3,x3);
  double phi = acos(max(-1, min(1, ((52.0 + pow(x3, 2.0) + pow(y3, 2.0)) / (28.0*sqrt(pow(x3, 2.0) + pow(y3, 2.0)))))));
  double alpha = acos(max(-1, min(1, (340.0 - pow(x3, 2.0) - pow(y3, 2.0)) / 336.0)));
  double theta1a = beta - phi;
  double theta2a = PI - alpha;
  double theta1b = beta + phi;
  double theta2b = alpha - PI;
  double theta3a = psi*DEG_TO_RAD - theta1a - theta2a;
  double theta3b = psi*DEG_TO_RAD - theta1b - theta2b;
  int shoulderAngleA = max(0, min(180, 7 + theta1a*RAD_TO_DEG));
  int elbowAngleA =   max(0, min(180, 180 - (90 + theta2a*RAD_TO_DEG)));
  int shoulderAngleB = max(0, min(180, 7 + theta1b*RAD_TO_DEG));
  int elbowAngleB =   max(0, min(180, 180 - (90 + theta2b*RAD_TO_DEG)));
  int wristAngleA = max(0, min(180, 90 + theta3a*RAD_TO_DEG));
  int wristAngleB = max(0, min(180, 90 + theta3b*RAD_TO_DEG));
  int baseAngle = theta4*RAD_TO_DEG + 88;
       if(x >= 0){ 
        shoulder.write(shoulderAngleB);
        elbow.write(elbowAngleB);
        wrist.write(wristAngleB);
        base.write(baseAngle);
       } else{
        shoulder.write(shoulderAngleA);
        elbow.write(elbowAngleA);
        wrist.write(wristAngleA); 
        base.write(baseAngle);       
       }
  //debug
  if(debug){ 
    if((millis() - debounce) > 1000){  
        //debg
        Serial.print("angulo ombro A: ");
        Serial.println(shoulderAngleA);
        Serial.print("angulo cotovelo A: ");
        Serial.println(elbowAngleA);
        Serial.print("angulo ombro B: ");
        Serial.println(shoulderAngleB);
        Serial.print("angulo cotovelo B: ");
        Serial.println(elbowAngleB);
        Serial.print("angulo punho A: ");
        Serial.println(wristAngleA);
        Serial.print("angulo punho B: ");
        Serial.println(wristAngleB);
        Serial.print("angulo base: ");
        Serial.println(baseAngle);
        debounce = millis();
    }
  }
}
 
