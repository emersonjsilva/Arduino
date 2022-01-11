/*-----------------------------------------------------------------------------/
 Project:         Automatic moving for robotic arm with servo SG90
 Date:            22/08/2021
 Programmers:     Emerson J. Silva, Elian P. S. Almeida
 Contact:         eng.emersonjsilva@gmail.com, elianprimo234@gmail.com

 Version:         1.0
 License:         GLP
 Aditional files: Axis.h, Axis.cpp, util.h 

 STRUCTURES:
 -----------
  MemmAxis -> Struct to store position for vector of motion
    Methods:
      setAxis -> set all axis
        Params:
          X, Y, Z, G
      get_() -> get one axis
        Return: axis position
  
 FUNCTIONS:
 ----------
  printLCD -> Function to print position of the axis

  movingAxis -> Run automatic moving
  
/----------------------------------------------------------------------------*/

#include <LiquidCrystal_I2C.h>
#include "Axis.h"
#include "util.h"

#define PROG 0
#define RUN 1
#define LAG 10
#define NUMCYCLES 50
#define ANLOGRES 1023

// memory to axis position //
typedef struct{
  int posX, posY, posZ, posG, vel;
  void setAxis(int posX_, int posY_, int posZ_, int posG_, int vel_){
    posX = posX_;
    posY = posY_;
    posZ = posZ_;
    posG = posG_;
    vel = vel_;
  }
  int getx(){ return posX; }
  int gety(){ return posY; }
  int getz(){ return posZ; }
  int getg(){ return posG; }
  int getv(){ return vel; }
}MemmAxis;

MemmAxis memmAxis[NUMCYCLES];

 //  Display  //
#define ADDR  0x27
#define COL  16
#define LIN  2  
LiquidCrystal_I2C lcd(ADDR, COL, LIN);

int statusProg = PROG;
String incomingByte;
int actualPos__ = 0;

int pinMemm = 2;
int pinExecuta = 4;
int pinSpin = 3;
int pinUpDown = 5;
int pinfwrdRev = 6;
int pinOpenClose = 9;

char mensagem[120];

Axis spin;
Axis upDown;
Axis fwrdRev;
Axis openClose;

TimerOn timer[3];
int cycle = 0;
int angularSpeed = 20;

void printLCD(int posx, int posy, int posz, int posg, char sts[2]){ 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X");
  lcd.setCursor(0, 1);
  lcd.print(posx);      
  lcd.setCursor(4, 0);
  lcd.print("Y");
  lcd.setCursor(4, 1);
  lcd.print(posy);          
  lcd.setCursor(8, 0);
  lcd.print("Z");
  lcd.setCursor(8, 1);
  lcd.print(posz);  
  lcd.setCursor(12, 0);
  lcd.print("G");
  lcd.setCursor(12, 1);
  lcd.print(posg);
  lcd.setCursor(15, 0);    
  lcd.print(sts[0]);
  lcd.setCursor(15, 1);    
  lcd.print(sts[1]);
}

void movingAxis(){
  int autoPos = 0;
  Serial.print("Executado posicao: ");
  Serial.println(autoPos);
  lcd.clear();
  lcd.print(" Exec pos: "); 

  float delta[4], steps, deltaMax = 0.0;
  for(int move_ = 0; move_ < actualPos__; move_++){

    int initialX = spin.getActualAngle();
    int initialY = upDown.getActualAngle(); 
    int initialZ = fwrdRev.getActualAngle();
    int initialG = openClose.getActualAngle();   
    
    delta[0] = memmAxis[move_].getx() - spin.getActualAngle();   
    delta[1] = memmAxis[move_].gety() - upDown.getActualAngle();  
    delta[2] = memmAxis[move_].getz() - fwrdRev.getActualAngle();  
    delta[3] = memmAxis[move_].getg() - openClose.getActualAngle();  
    angularSpeed = memmAxis[move_].getv();

    for(int i = 0; i < 4; i++){
        if(abs(delta[i]) > deltaMax){
          deltaMax = abs(delta[i]);
        }
    }
    for(int i = 0; i < 4; i++) Serial.println(delta[i]);
    steps = (1000 * abs(deltaMax)) / (angularSpeed * LAG);
    
    Serial.println(steps);
    for(int step_ = 0; step_ < steps; step_++){
      int movx = initialX + (int)(delta[0]*step_+1)/(steps * 1.f);
      spin.absoluteMoving(movx);
      int movy = initialY + (int)(delta[1]*step_+1)/(steps * 1.f);
      upDown.absoluteMoving(movy);
      int movz = initialZ + (int)(delta[2]*step_+1)/(steps * 1.f);
      fwrdRev.absoluteMoving(movz);
      int movg = initialG + (int)(delta[3]*step_+1)/(steps * 1.f);
      openClose.absoluteMoving(movg); 
      if( step_ % 50 == 0 ){
        int difX = abs(memmAxis[move_].getx() - spin.getActualAngle());    
        int difY = abs(memmAxis[move_].gety() - upDown.getActualAngle());    
        int difZ = abs(memmAxis[move_].getz() - fwrdRev.getActualAngle());    
        int difG = abs(memmAxis[move_].getg() - openClose.getActualAngle()); 
        printLCD(difX, difY, difZ, difG, "RN");
      }
      delay(LAG);
    }
  }
}

void setup() {  
  // Memoriza movimentos //
  pinMode(pinMemm, INPUT_PULLUP);
  // Executa movimentos //
  pinMode(pinExecuta, INPUT_PULLUP);
  
  spin.attachServo(pinSpin);
  upDown.attachServo(pinUpDown);
  fwrdRev.attachServo(pinfwrdRev);
  openClose.attachServo(pinOpenClose);
 
  // Display //
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Idle");
  lcd.setCursor(6, 1);
  lcd.print(":)");
  
  delay(1000);
  Serial.begin(9600);
  //Axis(int anlgInitialValue_, float initialAngle_,,  float angleMin_, float angleMax_, float kIncrement_, int deadBand_, float analogChannelResol_)
  spin.axisSet(analogRead(0), 90, 0, 180, 3, 1, ANLOGRES);//, pinSpin);
  upDown.axisSet(analogRead(1), 100, 70, 150, 3, 1, ANLOGRES);
  fwrdRev.axisSet(analogRead(2), 80, 20, 80, 3, 1, ANLOGRES);
  openClose.axisSet(analogRead(3), 120, 100, 180, 3, 1, ANLOGRES);
  delay(1000); 
}

void loop() {
  switch(statusProg){
    case RUN:
       movingAxis();      
      statusProg = PROG;
      break;
    case PROG:
      spin.incrementalMoving(analogRead(0)); 
      upDown.incrementalMoving(analogRead(1));   
      fwrdRev.incrementalMoving(analogRead(2));   
      openClose.incrementalMoving(analogRead(3)); 
      delay(50);

      if(cycle == 0){
        printLCD(spin.getActualAngle(), upDown.getActualAngle(), fwrdRev.getActualAngle(), openClose.getActualAngle(), "PG");
      }

      // Funcao para guardar valor atual
      if (Serial.available() > 0) {
        // read buffer received:
        incomingByte = Serial.readString();  
        String xval = splitString__(incomingByte, ' ', 0);
        String yval = splitString__(incomingByte, ' ', 1);
        String zval = splitString__(incomingByte, ' ', 2);
        String gval = splitString__(incomingByte, ' ', 3);       
        String vel = splitString__(incomingByte, ' ', 4); 
        memmAxis[actualPos__].setAxis(xval.toInt(), yval.toInt(), zval.toInt(), gval.toInt(), vel.toInt()); 
  
        lcd.clear();
        lcd.print("Save step: "); 
        lcd.setCursor(14, 0);
        lcd.print(actualPos__);
        delay(500);
        printLCD(memmAxis[actualPos__].getx(), memmAxis[actualPos__].gety(), memmAxis[actualPos__].getz(), memmAxis[actualPos__].getg(), "PG");
        
        delay(500);       
        actualPos__++;
        // responde com o dado recebido:
        Serial.println(incomingByte);
      }else if(timer[0].timerOnONS(500, !digitalRead(pinMemm))){
        memmAxis[actualPos__].setAxis(spin.getActualAngle(), upDown.getActualAngle(), fwrdRev.getActualAngle(), openClose.getActualAngle(), 20);      
        Serial.print("Position Memorized: ");
        Serial.println(actualPos__);
        Serial.println(spin.getActualAngle());
        Serial.println(upDown.getActualAngle());
        Serial.println(fwrdRev.getActualAngle());
        Serial.println(openClose.getActualAngle());
        
        lcd.clear();
        lcd.print("Save step: "); 
        lcd.setCursor(14, 0);
        lcd.print(actualPos__);            
        delay(1000);
        printLCD(memmAxis[actualPos__].getx(), memmAxis[actualPos__].getx(), memmAxis[actualPos__].getx(), memmAxis[actualPos__].getx(), "PG");
        
        delay(1000); 
        actualPos__++;         
      }
      if (actualPos__ >= NUMCYCLES){
        actualPos__ = NUMCYCLES - 1;    
      }        
      break;
  }; //end Switch
  
  if(timer[1].timerOnONS(5000, !digitalRead(pinExecuta))){   
    if(statusProg == PROG){
      statusProg = RUN;      
//     autoPos = 0;
      Serial.println("Moving");
    }
    else {
      statusProg = PROG;
      Serial.println("PROG");    
      lcd.clear();
      lcd.print(" PROG ");   
    }
  }
  cycle++;
  if(cycle > 10) cycle = 0;
  delay (LAG);  
};
