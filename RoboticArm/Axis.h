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

#ifndef AXIS_H
#define AXIS_H
#include <Servo.h>
#include "Arduino.h"

class Axis{
	private:
    float initialAngle;
    float actualAngle;
		int anlgInitialValue;
		float angleMin;
		float angleMax;
    float kIncrement;
		float deadBand;
    int analogChannelResol;
    int servoAttachPin;
    char msg[128];
    float angIncrement;
    Servo servo;
		
	public:
    Axis();
    void axisSet(int anlgInitialValue_, float initialAngle_, float angleMin_, float angleMax_, float kIncrement_, int deadBand_, float analogChannelResol_);
    void incrementalMoving(int anlgInput);
		void absoluteMoving(int angle_);
		int getActualAngle();
    void attachServo(int pin_);
};
#endif
