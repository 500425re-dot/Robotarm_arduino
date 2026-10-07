/*
ao mover o joystici o angulo é aumentado e gravado
mover um pouco aumenta um puco e mover bastante aumentar bastante 
    fazer que a varivael que aumenta é tipo o valor do pot * 0.1
ao apertar 1 vez o botao, os controles dos motoers mudan (qual motores)
os angulos gravados dos motores sao usados para restringir limites fisicos
no display, aparecera o angulo de cada motor, ao aparetra  o botao aparecerá

joystick- center values and ranges: center-90, range 1-179
movimento: ao chegar em 179 ou 1, aumenta/dimini o anglo e se segurar vai mais rapido
*/

#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
#include <Servo.h>

#define endereco  0x27 // Endereços comuns: 0x27, 0x3F
#define colunas   16
#define linhas    2
#define vrx A0
#define vry A1 
#define sw 7
#define but 8
#define servo1 3
#define servo2 5
#define servo3 6
#define servo4 9
#define servo5 10

#define baseHome 90
#define shoulderHome 100
#define elbowHome 170
#define wristHome 90 

bool swState;
bool swStateAnt;
bool butState;
bool butStateAnt;
int xState;
int yState;
float baseAngle = baseHome; 
float shoulderAngle = shoulderHome; 
float elbowAngle = elbowHome;  
float wristAngle = wristHome;
int gripperAngle = 160;
float xAngle;
float yAngle;
float xAngle2;
float yAngle2;
float angle3 = 50;
unsigned long debounce;
unsigned long debounce2;

Servo shoulderServo;
Servo baseServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;
LiquidCrystal_I2C lcd(endereco, colunas, linhas);

void setup() {
  // put your setup code here, to run once:
  lcd.init(); // begins communication with display
  lcd.backlight(); // turn display light on
  lcd.print(" -Bem vindo-");
  lcd.setCursor(0, 1); //set cursor on line 2
  lcd.print("Robot arm v1");
  delay(1500);
  lcd.clear(); // cleans the display

    pinMode(vrx, INPUT);
    pinMode(vry, INPUT);
    pinMode(sw, INPUT_PULLUP);
    pinMode(but, INPUT_PULLUP);
    shoulderServo.attach(servo2);
    baseServo.attach(servo3);
    elbowServo.attach(servo1);
    wristServo.attach(servo5);
    gripperServo.attach(servo4);
    Serial.begin(9600);
    
    //home position
    baseServo.write(baseAngle);
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
    wristServo.write(wristAngle);
    gripperServo.write(gripperAngle);
}

void loop() {
  // put your main code here, to run repeatedly:
    xState = map(analogRead(vrx), 0, 1023, 0, 180);
    yState = map(analogRead(vry), 0, 1023, 0, 180);
    
    //control with push buttons
    if((millis() - debounce) > 250){ 
      if(!digitalRead(but) && butStateAnt){ 
        butState = !butState;
        debounce = millis();
      }
    }
    if((millis() - debounce2) > 250){ 
      if(!digitalRead(sw) && swStateAnt){ 
        swState = !swState;
        debounce2 = millis();
      }
    }
    
    if(swState){ 
      xState > 100? angle3+=5 : (xState < 85? angle3-=5 : true);
      yState > 100? angle3++ : (yState < 85? angle3-- : true);
    }else{ 
      if(!butState){ 
        xState > 100? (xState > 170? xAngle+=5 : xAngle+=0.2) : (xState < 85? (xState < 10? xAngle-=5 : xAngle-=0.2) : true);
        yState > 100? (yState > 170? yAngle+=5 : yAngle+=0.2) : (yState < 85? (yState < 10? yAngle-=5 : yAngle-=0.2) : true);
      }else{
         xState > 100? (xState > 170? xAngle2+=5 : xAngle2+=0.2) : (xState < 85? (xState < 10? xAngle2-=5 : xAngle2-=0.2) : true);
         yState > 100? (yState > 170? yAngle2+=5 : yAngle2+=0.2) : (yState < 85? (yState < 10? yAngle2-=5 : yAngle2-=0.2) : true);
      }
    }

    //debug
    Serial.print("angulo x: ");
    Serial.println(xAngle);
    Serial.print("angulo y: ");
    Serial.println(yAngle);
    Serial.print("angulo x 2: ");
    Serial.println(xAngle2);
    Serial.print("angulo y 2: ");
    Serial.println(yAngle2);
    Serial.print("joy x: ");
    Serial.println(xState);
    Serial.print("joy y: ");
    Serial.println(yState);
    Serial.print(" angle 3 gripper:  ");
    Serial.println(angle3);
    Serial.println(swState);
    
    
    //  joints control
    if((baseHome - xAngle) < 0){
      xAngle= baseHome;
    } else if((baseHome - xAngle) > 180){
        xAngle = -baseHome;
    }
    if((shoulderHome - yAngle) < 0){
      yAngle=shoulderHome;
    } else if((shoulderHome - yAngle) > 180){
        yAngle = -shoulderHome;
    }
    if((elbowHome + xAngle2) < 0){
      xAngle2=-elbowHome;
    } else if((elbowHome + xAngle2) > 180){
        xAngle2 = 10;
    }
    if((wristHome + yAngle2) < 0){
      yAngle2=-wristHome;
    } else if((wristHome + yAngle2) > 180){
        yAngle2 = wristHome;
    }
      baseServo.write(baseHome - xAngle);
      shoulderServo.write(shoulderHome - yAngle);
      elbowServo.write(elbowHome + xAngle2);
      wristServo.write(wristHome + yAngle2);
    
    // gripper control angle
    angle3 > 150? angle3 = 150: (angle3 < 50? angle3 = 50: true); 
    gripperServo.write(angle3);
    
    if(butStateAnt && swStateAnt){
      baseServo.write(baseHome);
      shoulderServo.write(shoulderHome);
      elbowServo.write(elbowHome);
      wristServo.write(wristHome);
    
      xAngle = yAngle = xAngle2 = yAngle2 = 0;
    }
    
    delay(15);
    butStateAnt = !digitalRead(but);
    swStateAnt = !digitalRead(sw);
}
