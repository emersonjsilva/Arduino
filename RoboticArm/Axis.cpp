/*-----------------------------------------------------------------------------/
 Project:         Automatic moving for robotic arm with servo SG90
 Date:            22/08/2021
 Programmers:     Emerson J. Silva, Elian P. S. Almeida
 Contact:         eng.emersonjsilva@gmail.com, elianprimo234@gmail.com

 Version:         1.0
 License:         GLP

 CLASS AXIS:
 ----------
  axisSet -> Setup parameters to each axis
    Params:
      int anlgInitialValue_     -> Initial analog value of input (AKA zero offset)
      float initialAngle_       -> Initial angle to positioning axis on init
      float angleMin_           -> Min range of move
      float angleMax_           -> Max range of move
      float kIncrement_         -> Constant to incremental moving
      int deadBand_             -> Dead band of analog input
      float analogChannelResol_ -> Resotulion of analog channel
      
  incrementalMoving -> incremental moviment using analog input
    Params: int anlgInput -> value of analog input

  absoluteMoving -> Move to desire angle
    Params: int angle_ -> Angle to move

  getActualAngle  -> Get actual angle of servo
    Return: Actual angle of servo

  attachServo -> attach servo to a pin
    Params: int pin_ -> pin to attach servo  
  
/----------------------------------------------------------------------------*/

#include "Axis.h"

Axis::Axis(){
	actualAngle = 0;
  initialAngle = 0;
	kIncrement = 10.0;
	angleMin = 0;
	angleMax = 180;
	deadBand = 20;
	analogChannelResol = 1023;
  anlgInitialValue = 0;
}

void Axis::axisSet(int anlgInitialValue_, float initialAngle_, float angleMin_, float angleMax_, float kIncrement_, int deadBand_, float analogChannelResol_){
  anlgInitialValue = anlgInitialValue_;
  initialAngle = initialAngle_;
	actualAngle = initialAngle_;
	kIncrement = kIncrement_;
	angleMin = angleMin_;
	angleMax = angleMax_;
	deadBand = deadBand_;
	analogChannelResol = analogChannelResol_;
  //servoAttachPin = pin_;
  //attachServo(pin_);
  absoluteMoving(initialAngle_);
}

void Axis::incrementalMoving(int anlgInput){
  //Filtro filtro;

  angIncrement = (kIncrement * (anlgInput - anlgInitialValue)) /  analogChannelResol;
  //Serial.println(actualAngle);

  if(abs(angIncrement) > deadBand){
      actualAngle += angIncrement;
      if(actualAngle < angleMin) actualAngle = angleMin;
      else if(actualAngle > angleMax) actualAngle = angleMax;
      
      absoluteMoving((int)actualAngle);    
  }
}

void Axis::absoluteMoving(int angle_){
  actualAngle = angle_;
  //analogWrite(servoAttachPin, (int)(actualAngle * 255 )/180);
  servo.write(actualAngle);
}

int Axis::getActualAngle(){
  //return actualAngle;
  return servo.read();
}

void Axis::attachServo(int pin_){
  servoAttachPin = pin_;
  //pinMode(pin_, OUTPUT);
  servo.attach(pin_);
};
